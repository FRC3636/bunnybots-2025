package com.frcteam3636.bunnybots2025.subsystems.shooter

import com.ctre.phoenix6.BaseStatusSignal
import com.frcteam3636.bunnybots2025.Robot
import com.frcteam3636.bunnybots2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.bunnybots2025.subsystems.drivetrain.FIELD_LAYOUT
import com.frcteam3636.bunnybots2025.subsystems.indexer.Indexer
import com.frcteam3636.bunnybots2025.subsystems.shooter.Shooter.Flywheels.lowerSetpoint
import com.frcteam3636.bunnybots2025.subsystems.shooter.Shooter.Flywheels.upperSetpoint
import com.frcteam3636.bunnybots2025.utils.math.*
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
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
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import kotlin.math.abs
import kotlin.math.pow

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
                val velocityDifference = inputs.topVelocity - upperSetpoint
                Logger.recordOutput("Shooter/Flywheels/Velocity Difference", velocityDifference)
                Logger.recordOutput(
                    "Shooter/Flywheels/At Desired Velocity",
                    abs(velocityDifference.inRPM()) < FLYWHEEL_VELOCITY_TOLERANCE.inRPM()
                )
                abs(velocityDifference.inRPM()) < FLYWHEEL_VELOCITY_TOLERANCE.inRPM()
            }

        val speedInterpolationTable = InterpolatingDoubleTreeMap()

        init {
            //FIXME plot points to create regression
            speedInterpolationTable.putVelocity(1.66.meters, 2650.rpm)
//            speedInterpolationTable.putVelocity(2.5.meters, 3400.rpm)
            speedInterpolationTable.putVelocity(3.4.meters, 4300.rpm)
        }

        private val inputs = LoggedFlywheelInputs()

        @Suppress("Unused")
        var sysID = SysIdRoutine(
            SysIdRoutine.Config(
                null,
                null,
                null,
                { state ->
                    Logger.recordOutput("SysIdTestState", state.toString())
                }
            ),
            SysIdRoutine.Mechanism(
                io::setVoltage,
                null, // recorded by URCL
                this
            )
        )

        @Suppress("unused")
        fun sysIdQuasistatic(direction: SysIdRoutine.Direction): Command = sysID.quasistatic(direction)

        @Suppress("unused")
        fun sysIdDynamic(direction: SysIdRoutine.Direction): Command = sysID.dynamic(direction)

        override fun periodic() {
            io.updateInputs(inputs)

            Logger.processInputs("Shooter/Flywheels", inputs)

            Logger.recordOutput("Shooter/Flywheels/upperSetpoint", upperSetpoint)
            Logger.recordOutput("Shooter/Flywheels/lowerSetpoint", lowerSetpoint)
            io.setVelocity(upperSetpoint, lowerSetpoint) // move this into commands?
        }

        fun idle(): Command =
            runEnd(
                {
                    upperSetpoint = -3.radiansPerSecond
                    lowerSetpoint = -3.radiansPerSecond
                },
                {
                    upperSetpoint = 0.radiansPerSecond
                    lowerSetpoint = 0.radiansPerSecond
                }
            )

        fun shoot(): Command =
            //TODO: Fix
            run {
                upperSetpoint = Pivot.target.profile.getVelocity()
                lowerSetpoint = Pivot.target.profile.getVelocity()
            }

        val signals: Array<BaseStatusSignal>
            get() = io.signals
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

        private val pivotDisabledAlert = Alert(
            "The shooter pivot has been disabled due to an error. To re-enable please restart robot code :3",
            Alert.AlertType.kError
        )

        init {
            //FIXME plot points to create regression
            angleInterpolationTable.putAngle(1.66.meters, 55.0.degrees)
//            angleInterpolationTable.putAngle(2.5.meters, 50.0.degrees)
            angleInterpolationTable.putAngle(3.4.meters, 40.0.degrees)

            mechanism.getRoot("Shooter Pivot", 50.0, 150.0).apply {
                append(pivotAngleLigament)
            }
        }

        override fun periodic() {
            io.updateInputs(inputs)
            Logger.processInputs("Shooter/Pivot", inputs)

            // the extra degree is to account for encoder noise
            if ((inputs.pivotAngle < 4.degrees || inputs.pivotAngle > 91.degrees) && !inputs.pivotDisabled && Robot.model != Robot.Model.SIMULATION) {
                io.disablePivot()
                pivotDisabledAlert.set(true)
            }

            pivotAngleLigament.angle = inputs.pivotAngle.inDegrees()
            Logger.recordOutput("Shooter/Pivot/Mechanism", mechanism)
            Logger.recordOutput("Shooter/Pivot/Active Profile", target)
            Logger.recordOutput("Shooter/Pivot/Reference", target.profile.getPosition())
        }

        val atDesiredPosition =
            Trigger {
                val difference = inputs.pivotAngle.inDegrees() - target.profile.getPosition().inDegrees()
                Logger.recordOutput("Shooter/Pivot/At Desired Position", abs(difference) < 3.0)
                abs(difference) < 3.0
            }

        val signals: Array<BaseStatusSignal>
            get() = io.signals

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

        fun idle(): Command =
            runEnd(
                {
                    if (Shooter.Flywheels.isDetected.asBoolean) {
                        io.setSpeed(-0.04)
                    } else {
                        io.setSpeed(0.0)
                    }
                },
                {
                    io.setSpeed(0.0)
                }
            )

        fun feed(interruptBehavior: Command.InterruptionBehavior = Command.InterruptionBehavior.kCancelSelf): Command =
            startEnd(
                {
                    io.setSpeed(0.1)
                },
                {
                    io.setSpeed(0.0)
                }
            ).withInterruptBehavior(interruptBehavior)

        fun backup(): Command =
            startEnd(
                {
                    io.setSpeed(-0.1)
                },
                {
                    io.setSpeed(0.0)
                }
            )
    }
}

data class ShooterProfile(
    val getPosition: () -> Angle,
    val getVelocity: () -> AngularVelocity
)

fun distanceToZoo(): Distance {
    val pettingZooTranslation = DriverStation.getAlliance()
        .orElse(DriverStation.Alliance.Blue)
        .zooTranslation
    val zooPose = Pose2d(
        pettingZooTranslation,
        Rotation2d.kZero
    )
    val distance = Drivetrain.estimatedPose.translation.getDistance(zooPose.translation).meters
    return distance
}

val pivotTunable = LoggedNetworkNumber("/Tuning/PivotTestAngle", 12.0)
val flywheelTunable = LoggedNetworkNumber("/Tuning/FlywheelTestSpeed", 0.0)


// should we move this inside the shooter object?
enum class Target(val profile: ShooterProfile) {
    AIM(
        ShooterProfile(
            {
                val distance = distanceToZoo().inMeters()
                val angle = ((-2.96479 * distance.pow(2)) + (6.38113 * distance) + 52.57708)
                MathUtil.clamp(angle, 12.0, 60.0).degrees
            }, {
                val distance = distanceToZoo().inMeters()
                val speed = ((61.57635 * distance.pow(2)) + (636.69951 * distance) + 1423.39901)
                MathUtil.clamp(speed, 0.0, 5500.0).rpm
            }
        )
    ),
    PETTINGZOO(
        ShooterProfile(
            {
                30.degrees
            },
            {
                1000.rpm
            }
        )
    ),
    STOWED(
        ShooterProfile(
            {
                12.degrees
            },
            {
                1000.rpm // dude idek what to set this to lmao
            }
        )
    ),
    TUNING(
        ShooterProfile(
            {
                pivotTunable.get().degrees
            },
            {
                flywheelTunable.get().rpm
            }
        ),
    )
}

val DriverStation.Alliance.zooTranslation: Translation2d
    get() = when (this) { // got these values from apriltag math
        DriverStation.Alliance.Red -> (FIELD_LAYOUT.getTagPose(5).get().translation.toTranslation2d() +
                FIELD_LAYOUT.getTagPose(7).get().translation.toTranslation2d()) / 2.0

        else -> (FIELD_LAYOUT.getTagPose(1).get().translation.toTranslation2d() +
                FIELD_LAYOUT.getTagPose(3).get().translation.toTranslation2d()) / 2.0
    }

internal val FLYWHEEL_VELOCITY_TOLERANCE = 2.radiansPerSecond
