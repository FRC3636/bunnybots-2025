package com.frcteam3636.bunnybots2025.subsystems.shooter

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.SignalLogger
import com.frcteam3636.bunnybots2025.Robot
import com.frcteam3636.bunnybots2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.bunnybots2025.utils.math.*
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d
import kotlin.math.atan

object Shooter {
    object Flywheel : Subsystem {
        private var io = when (Robot.model) {
            Robot.Model.COMPETITION -> FlywheelIOReal()
            Robot.Model.SIMULATION -> FlywheelIOSim()
        }

        private var upperSetpoint = RadiansPerSecond.zero()!!
        private var lowerSetpoint = RadiansPerSecond.zero()!!

        private val inputs = LoggedFlywheelInputs()

        private var upperPidController = PIDController(Constants.UPPER_PID_GAINS)
        private var lowerPidController = PIDController(Constants.LOWER_PID_GAINS)
        private var upperFFController = SimpleMotorFeedforward(Constants.UPPER_FF_GAINS)
        private var lowerFFController = SimpleMotorFeedforward(Constants.LOWER_FF_GAINS)

        @Suppress("Unused")
        var sysID = SysIdRoutine(
            SysIdRoutine.Config(
                0.5.voltsPerSecond, 2.volts, null, {
                    SignalLogger.writeString("state", it.toString())
                }), SysIdRoutine.Mechanism(
                io::setVoltage,
                null,
                this,
            )
        )

        override fun periodic() {
            io.updateInputs(inputs)

            Logger.processInputs("Shooter/Flywheels", inputs)

            io.setVoltage(
                (upperFFController.calculate(upperSetpoint.inRadiansPerSecond()) +
                        upperPidController.calculate(
                            inputs.topVelocity.inRadiansPerSecond(),
                            upperSetpoint.inRadiansPerSecond()
                        )).volts,
                (lowerFFController.calculate(lowerSetpoint.inRadiansPerSecond()) +
                        lowerPidController.calculate(
                            inputs.bottomVelocity.inRadiansPerSecond(),
                            lowerSetpoint.inRadiansPerSecond()
                        )).volts,
            )
        }

        fun pulse(): Command =
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

        private var currentTarget = Target.STOWED

        var mechanism = LoggedMechanism2d(100.0, 200.0)
        var pivotAngleLigament = LoggedMechanismLigament2d("Pivot Ligament", 50.0, 180.0, 5.0, Color8Bit(Color.kGreen))

        init {
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
                io.turnToAngle(target.profile.position())
            }

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
                        atan(pettingZooTranslation.z / distance).degrees
                    }
                )
            ),
            PETTINGZOO(
                PivotProfile(
                    {
                        45.degrees
                    }
                )
            ),
            STOWED(
                PivotProfile(
                    {
                        Degrees.zero()!!
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

        val isDetected: Boolean
            get() {
                return inputs.isDetected
            }

        override fun periodic() {
            io.updateInputs(inputs)
            Logger.processInputs("Shooter/Feeder", inputs)
        }

        fun intake(): Command =
            startEnd(
                {
                    io.setSpeed(0.7)
                },
                {
                    io.setSpeed(0.0)
                }
            )

        fun getStatusSignals(): MutableList<BaseStatusSignal> {
            return io.getStatusSignals()
        }
    }
}

data class PivotProfile(
    val position: () -> Angle
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