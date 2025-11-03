package com.frcteam3636.bunnybots2025.subsystems.shooter

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.SignalLogger
import com.frcteam3636.bunnybots2025.Robot
import com.frcteam3636.bunnybots2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.bunnybots2025.subsystems.drivetrain.FIELD_LAYOUT
import com.frcteam3636.bunnybots2025.subsystems.shooter.Shooter.Flywheels.speedInterpolationTable
import com.frcteam3636.bunnybots2025.subsystems.shooter.Shooter.Pivot.angleInterpolationTable
import com.frcteam3636.bunnybots2025.subsystems.shooter.Shooter.Pivot.mechanism
import com.frcteam3636.bunnybots2025.subsystems.shooter.Shooter.Pivot.pivotAngleLigament
import com.frcteam3636.bunnybots2025.utils.math.*
import com.pathplanner.lib.util.FlippingUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.Alert
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

        private var setpoint = RadiansPerSecond.zero()!!

        val isDetected: Trigger =
            Trigger {
                inputs.isDetected
            }

        val atDesiredVelocity =
            Trigger {
                val velocityDifference = inputs.topVelocity - setpoint
                Logger.recordOutput("Shooter/Flywheels/Velocity Difference", velocityDifference)
                Logger.recordOutput(
                    "Shooter/Flywheels/At Desired Velocity",
                    velocityDifference < FLYWHEEL_VELOCITY_TOLERANCE
                )
                velocityDifference < FLYWHEEL_VELOCITY_TOLERANCE
            }

        val speedInterpolationTable = InterpolatingDoubleTreeMap()

        init {
            //FIXME plot points to create regression
            speedInterpolationTable.putVelocity(5.0.meters, 60.0.radiansPerSecond)
            speedInterpolationTable.putVelocity(7.5.meters, 45.0.radiansPerSecond)
            speedInterpolationTable.putVelocity(10.0.meters, 30.0.radiansPerSecond)
            speedInterpolationTable.putVelocity(12.5.meters, 25.0.radiansPerSecond)
        }

        private val inputs = LoggedFlywheelInputs()

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

            Logger.recordOutput("Shooter/Flywheels/Setpoint", setpoint)
            io.setVelocity(setpoint) // move this into commands?
        }

        fun idle(): Command =
            startEnd(
                {
                    setpoint = 1.radiansPerSecond
                },
                {
                    setpoint = 0.radiansPerSecond
                }
            )

        fun shoot(): Command =
            run {
                setpoint = Pivot.target.profile.getVelocity()
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

        val angleInterpolationTable = InterpolatingDoubleTreeMap()

        private val pivotDisabledAlert = Alert("The shooter pivot has been disabled due to an error. To re-enable please restart robot code",
            Alert.AlertType.kError)

        init {
            //FIXME plot points to create regression
            angleInterpolationTable.putAngle(5.0.meters, 60.0.degrees)
            angleInterpolationTable.putAngle(7.5.meters, 45.0.degrees)
            angleInterpolationTable.putAngle(10.0.meters, 30.0.degrees)
            angleInterpolationTable.putAngle(12.5.meters, 25.0.degrees)

            mechanism.getRoot("Shooter Pivot", 50.0, 150.0).apply {
                append(pivotAngleLigament)
            }
        }

        override fun periodic() {
            io.updateInputs(inputs)
            Logger.processInputs("Shooter/Pivot", inputs)

            // the extra degree is to account for encoder noise
            if ((inputs.pivotAngle < (-1).degrees || inputs.pivotAngle > 91.degrees) && !inputs.pivotDisabled) {
                io.disablePivot()
                pivotDisabledAlert.set(true)
            }

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
            run {
                if (!inputs.pivotDisabled)
                    io.turnToAngle(target.profile.getPosition())
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

fun distanceToZoo(): Distance {
    val pettingZooTranslation = DriverStation.getAlliance()
        .orElse(DriverStation.Alliance.Blue)
        .zooTranslation
    val zooPose = Pose2d(
        pettingZooTranslation,
        Rotation2d()
    )
    val distance = Drivetrain.estimatedPose.translation.getDistance(zooPose.translation).meters
    return distance
}


// should we move this inside the shooter object?
enum class Target(val profile: PivotProfile) {
    AIM(
        PivotProfile(
            {
                angleInterpolationTable.getAngle(distanceToZoo())
            }, {
                speedInterpolationTable.getVelocity(distanceToZoo())
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

val DriverStation.Alliance.zooTranslation: Translation2d
    get() = when (this) { // got these values from apriltag math
        DriverStation.Alliance.Blue -> Translation2d(
            12.9327783.meters,
            4.0132127.meters,
        )
        else -> Translation2d(
            FIELD_LAYOUT.fieldLength.meters - 12.9327783.meters,
            4.0132127.meters
        )
    }

internal val FLYWHEEL_VELOCITY_TOLERANCE = 2.radiansPerSecond