package com.frcteam3636.bunnybots2025.subsystems.shooter

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.SignalLogger
import com.frcteam3636.bunnybots2025.Robot
import com.frcteam3636.bunnybots2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.bunnybots2025.utils.math.*
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d
import kotlin.math.absoluteValue

object Shooter {
    object Flywheels : Subsystem {
        private var io = when (Robot.model) {
            Robot.Model.COMPETITION -> FlywheelIOReal()
            Robot.Model.SIMULATION -> FlywheelIOSim()
        }

        private var upperSetpoint = RadiansPerSecond.zero()!!
        private var lowerSetpoint = RadiansPerSecond.zero()!!

        val isDetected: Trigger =
            Trigger {
                inputs.isDetected
            }

        val atDesiredVelocity =
            Trigger {
                val velocityDifference = (inputs.topVelocity.minus(upperSetpoint).baseUnitMagnitude().absoluteValue)
                Logger.recordOutput("Shooter/Flywheels/Velocity Difference", velocityDifference)
                Logger.recordOutput(
                    "Shooter/Flywheels/At Desired Velocity",
                    velocityDifference < FLYWHEEL_VELOCITY_TOLERANCE.baseUnitMagnitude()
                )
                velocityDifference < FLYWHEEL_VELOCITY_TOLERANCE.baseUnitMagnitude()
            }

        private val inputs = LoggedFlywheelInputs()

        private var upperPidController = PIDController(Constants.UPPER_PID_GAINS)
        private var lowerPidController = PIDController(Constants.LOWER_PID_GAINS)
        private var upperFFController = SimpleMotorFeedforward(Constants.UPPER_FF_GAINS)
        private var lowerFFController = SimpleMotorFeedforward(Constants.LOWER_FF_GAINS)

        @Suppress("Unused")
        var sysID = SysIdRoutine(
            SysIdRoutine.Config(
                0.5.voltsPerSecond, 2.volts, null
            ) {
                SignalLogger.writeString("state", it.toString())
            }, SysIdRoutine.Mechanism(
                io::setVoltage,
                null,
                this,
            )
        )

        override fun periodic() {
            io.updateInputs(inputs)

            Logger.processInputs("Shooter/Flywheels", inputs)

            Logger.recordOutput("Shooter/Flywheels/Upper Setpoint", upperSetpoint)
            Logger.recordOutput("Shooter/Flywheels/Lower Setpoint", lowerSetpoint)

            val upperVoltage = (upperFFController.calculate(upperSetpoint.inRadiansPerSecond()) +
                    upperPidController.calculate(
                        inputs.topVelocity.inRadiansPerSecond(),
                        upperSetpoint.inRadiansPerSecond()
                    )).volts

            val lowerVoltage = (lowerFFController.calculate(lowerSetpoint.inRadiansPerSecond()) +
                    lowerPidController.calculate(
                        inputs.bottomVelocity.inRadiansPerSecond(),
                        lowerSetpoint.inRadiansPerSecond()
                    )).volts

            Logger.recordOutput("Shooter/Flywheels/Upper Calculated Voltage", upperVoltage)
            Logger.recordOutput("Shooter/Flywheels/Lower Calculated Voltage", lowerVoltage)

            io.setVoltage(
                upperVoltage,
                lowerVoltage
            )
        }

        fun idle(): Command =
            startEnd(
                {
                    upperSetpoint = 1.radiansPerSecond
                    lowerSetpoint = 1.radiansPerSecond
                },
                {
                    upperSetpoint = 0.radiansPerSecond
                    lowerSetpoint = 0.radiansPerSecond
                }
            )

        fun shoot(): Command =
            run {
                upperSetpoint = Pivot.target.profile.getVelocity()
                lowerSetpoint = Pivot.target.profile.getVelocity()
            }

        object Constants {
            val UPPER_PID_GAINS = PIDGains()
            val LOWER_PID_GAINS = PIDGains()
            val UPPER_FF_GAINS = MotorFFGains()
            val LOWER_FF_GAINS = MotorFFGains()
        }
    }

    object Pivot : Subsystem {
        private var io = when (Robot.model) {
            Robot.Model.COMPETITION -> PivotIOReal()
            Robot.Model.SIMULATION -> PivotIOSim()
        }

        private val inputs = LoggedPivotInputs()

        var target = Target.STOWED

        var mechanism = LoggedMechanism2d(100.0, 200.0)
        var pivotAngleLigament = LoggedMechanismLigament2d("Pivot Ligament", 50.0, 180.0, 5.0, Color8Bit(Color.kGreen))

        private val interpolationTable = InterpolatingDoubleTreeMap()

        init {
            //FIXME plot points to create regression
            interpolationTable.put(5.0, 60.0)
            interpolationTable.put(7.5, 45.0)
            interpolationTable.put(10.0, 30.0)
            interpolationTable.put(12.5, 25.0)

            mechanism.getRoot("Shooter/Pivot", 50.0, 150.0).apply {
                append(pivotAngleLigament)
            }
        }

        override fun periodic() {
            io.updateInputs(inputs)

            Logger.processInputs("Shooter/Pivot", inputs)

            pivotAngleLigament.angle = inputs.pivotAngle.inDegrees()
            Logger.recordOutput("Shooter/Pivot/Mechanism", mechanism)
        }

        fun getStatusSignals(): MutableList<BaseStatusSignal> {
            return io.getStatusSignals()
        }

        fun setTarget(target: Target): Command =
            runOnce {
                Pivot.target = target
            }

        fun moveToActiveTarget(): Command =
            run { io.turnToAngle(target.profile.getPosition()) }

        enum class Target(val profile: PivotProfile) {
            AIM(
                PivotProfile(
                    {
                        val pettingZooTranslation = DriverStation.getAlliance()
                            .orElse(DriverStation.Alliance.Blue)
                            .zooTranslation
                        val zooPose = Pose2d(
                            pettingZooTranslation.toTranslation2d(),
                            Rotation2d()
                        )
                        val distance = zooPose.translation.minus(Drivetrain.estimatedPose.translation).norm
                        interpolationTable.get(distance).degrees
                    }, {
                        5000.rpm
                    }
                )
            ),
            PETTINGZOO(
                PivotProfile(
                    {
                        45.degrees
                    },
                    {
                        1000.rpm
                    }
                )
            ),
            STOWED(
                PivotProfile(
                    {
                        Degrees.zero()!!
                    },
                    {
                        1000.rpm // dude idek what to set this to lmao
                    }
                )
            )
        }
    }

    object Feeder : Subsystem {
        private var io = when (Robot.model) {
            Robot.Model.COMPETITION -> FeederIOReal()
            Robot.Model.SIMULATION -> FeederIOSim()
        }

        private val inputs = LoggedFeederInputs()

        override fun periodic() {
            io.updateInputs(inputs)
            Logger.processInputs("Shooter/Feeder", inputs)
        }

        fun feed(interruptBehavior: Command.InterruptionBehavior = Command.InterruptionBehavior.kCancelSelf): Command =
            startEnd(
                {
                    io.setSpeed(0.7)
                },
                {
                    io.setSpeed(0.0)
                }
            ).withInterruptBehavior(interruptBehavior)
    }
}

data class PivotProfile(
    val getPosition: () -> Angle,
    val getVelocity: () -> AngularVelocity
)

val DriverStation.Alliance.zooTranslation: Translation3d
    get() = when (this) { // got these values from field CAD
        DriverStation.Alliance.Blue -> Translation3d(
            Units.inchesToMeters(240.0),
            Units.inchesToMeters(180.0),
            Units.inchesToMeters(48.125)
        )

        else -> Translation3d(
            Units.inchesToMeters(600.0),
            Units.inchesToMeters(180.0),
            Units.inchesToMeters(48.125)
        )
    }

internal val FLYWHEEL_VELOCITY_TOLERANCE = 2.radiansPerSecond