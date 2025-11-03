package com.frcteam3636.bunnybots2025.subsystems.drivetrain

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.SignalLogger
import com.frcteam3636.bunnybots2025.CTREDeviceId
import com.frcteam3636.bunnybots2025.Robot
import com.frcteam3636.bunnybots2025.subsystems.drivetrain.Drivetrain.Constants.BRAKE_POSITION
import com.frcteam3636.bunnybots2025.subsystems.drivetrain.Drivetrain.Constants.DRIVE_BASE_RADIUS
import com.frcteam3636.bunnybots2025.subsystems.drivetrain.Drivetrain.Constants.FREE_SPEED
import com.frcteam3636.bunnybots2025.subsystems.drivetrain.Drivetrain.Constants.JOYSTICK_DEADBAND
import com.frcteam3636.bunnybots2025.subsystems.drivetrain.Drivetrain.Constants.MODULE_POSITIONS
import com.frcteam3636.bunnybots2025.subsystems.drivetrain.Drivetrain.Constants.PATH_FOLLOWING_ROTATION_GAINS
import com.frcteam3636.bunnybots2025.subsystems.drivetrain.Drivetrain.Constants.PATH_FOLLOWING_TRANSLATION_GAINS
import com.frcteam3636.bunnybots2025.subsystems.drivetrain.Drivetrain.Constants.POLAR_DRIVING_GAINS
import com.frcteam3636.bunnybots2025.subsystems.drivetrain.Drivetrain.Constants.ROTATION_SENSITIVITY
import com.frcteam3636.bunnybots2025.subsystems.drivetrain.Drivetrain.Constants.TRANSLATION_SENSITIVITY
import com.frcteam3636.bunnybots2025.subsystems.drivetrain.Drivetrain.Constants.WHEEL_COF
import com.frcteam3636.bunnybots2025.utils.fieldRelativeTranslation2d
import com.frcteam3636.bunnybots2025.utils.math.*
import com.frcteam3636.bunnybots2025.utils.swerve.Corner
import com.frcteam3636.bunnybots2025.utils.swerve.PerCorner
import com.frcteam3636.bunnybots2025.utils.swerve.cornerStatesToChassisSpeeds
import com.frcteam3636.bunnybots2025.utils.swerve.toCornerSwerveModuleStates
import com.frcteam3636.bunnybots2025.utils.translation2d
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.commands.PathfindingCommand
import com.pathplanner.lib.config.ModuleConfig
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.pathfinding.Pathfinding
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.littletonrobotics.junction.Logger
import java.util.Optional
import kotlin.jvm.optionals.getOrNull
import kotlin.math.*

/** A singleton object representing the drivetrain. */
object Drivetrain : Subsystem {
    private val io = when (Robot.model) {
        Robot.Model.SIMULATION -> DrivetrainIOSim()
        Robot.Model.COMPETITION -> DrivetrainIOReal(
            MODULE_POSITIONS.zip(Constants.MODULE_CAN_IDS)
                .map { (corner, ids) ->
                    val (driveId, turnId, encoderId) = ids
                    Mk5nSwerveModule(
                        DrivingTalon(driveId),
                        TurningTalon(turnId, encoderId, corner.magnetOffset),
                        corner.position.rotation
                    )
                })
    }
    val inputs = LoggedDrivetrainInputs()

    private val limiter = SlewRateLimiter(0.05)
    private var wheelRadiusModuleStates = DoubleArray(4)
    private var wheelRadiusLastAngle = Rotation2d()
    private var wheelRadiusGyroDelta = 0.0
    fun calculateWheelRadius(): Command = Commands.parallel(
        Commands.sequence(
            Commands.runOnce({
                Logger.recordOutput("Drivetrain/Wheel Radius Calculated/Running", true)
                limiter.reset(0.0)
            }),
            Commands.run({
                val speed = limiter.calculate(0.1)
                driveWithoutDeadband(Translation2d(), Translation2d(0.0, speed))
            }, Drivetrain)
        ),
        Commands.sequence(
            // Wait for modules to orient
            Commands.waitSeconds(1.0),
            Commands.runOnce({
                for (i in 0..3) {
                    wheelRadiusModuleStates[i] = io.modules.toTypedArray()[i].positionRad.inRadians()
                }
                wheelRadiusLastAngle = inputs.gyroRotation
                wheelRadiusGyroDelta = 0.0
            }),
            Commands.run({
                val rotation = inputs.gyroRotation
                wheelRadiusGyroDelta += abs(rotation.minus(wheelRadiusLastAngle).radians)
                wheelRadiusLastAngle = rotation
                Logger.recordOutput("Drivetrain/Wheel Radius Calculated/Gyro Delta", wheelRadiusGyroDelta)
            })
                .finallyDo { ->
                    var wheelDelta = 0.0
                    // Someone give me a better way to do this
                    for (i in 0..3) {
                        wheelDelta += abs(io.modules.toTypedArray()[i].positionRad.inRadians() - wheelRadiusModuleStates[i]) / 4.0
                        Logger.recordOutput(
                            "Drivetrain/Wheel Radius Calculated/Initial Wheel Position Rad/$i",
                            wheelRadiusModuleStates[i]
                        )
                        Logger.recordOutput(
                            "Drivetrain/Wheel Radius Calculated/Final Wheel Position Rad/$i",
                            io.modules.toTypedArray()[i].positionRad.inRadians()
                        )
                    }
                    val wheelRadius = ((wheelRadiusGyroDelta * DRIVE_BASE_RADIUS) / wheelDelta)
                    Logger.recordOutput("Drivetrain/Wheel Radius Calculated/Drive Base Radius", DRIVE_BASE_RADIUS)
                    Logger.recordOutput("Drivetrain/Wheel Radius Calculated/Wheel Delta", wheelDelta)
                    Logger.recordOutput("Drivetrain/Wheel Radius Calculated/Meters", wheelRadius)
                    Logger.recordOutput("Drivetrain/Wheel Radius Calculated/Inches", wheelRadius.meters.inInches())
                    Logger.recordOutput("Drivetrain/Wheel Radius Calculated/Running", false)
                }
        )
    )

    private val mt2Algo = LimelightAlgorithm.MegaTag2({
        poseEstimator.estimatedPosition.rotation
    }, {
        inputs.gyroVelocity
    })

    private var rawGyroRotation = Rotation2d.kZero

    // someone please give me a better way to do this
    val lastModulePositions = arrayOf(
        SwerveModulePosition(),
        SwerveModulePosition(),
        SwerveModulePosition(),
        SwerveModulePosition()
    )

    private val absolutePoseIOs = when (Robot.model) {
        Robot.Model.SIMULATION -> mapOf(
            "Limelight" to CameraSimPoseProvider("limelight", Transform3d()),
        )

        else -> mapOf(
//            "Limelight Rear" to LimelightPoseProvider(
//                "limelight-rear",
//                algorithm = mt2Algo
//            ),
        )
    }.mapValues { Pair(it.value, AbsolutePoseProviderInputs()) }

    /** Helper for converting a desired drivetrain velocity into the speeds and angles for each swerve module */
    private val kinematics =
        SwerveDriveKinematics(
            *MODULE_POSITIONS
                .map { it.position.translation }
                .toTypedArray()
        )

    /** Helper for estimating the location of the drivetrain on the field */
    val poseEstimator =
        SwerveDrivePoseEstimator(
            kinematics, // swerve drive kinematics
            inputs.gyroRotation, // initial gyro rotation
            inputs.measuredPositions.toTypedArray(), // initial module positions
            Pose2d(), // initial pose
            VecBuilder.fill(0.02, 0.02, 0.005),
            // Overwrite each measurement
            VecBuilder.fill(0.0, 0.0, 0.0)
        )

    /** Whether every sensor used for pose estimation is connected. */
    val allPoseProvidersConnected
        get() = absolutePoseIOs.values.all { it.second.connected }

    init {
        Pathfinding.setPathfinder(
            LocalADStarAK()
        )

        val pathPlannerConfig = RobotConfig(
            60.kilograms, // FIXME: weigh the robot
            6.883.kilogramSquareMeters, // FIXME: calculate with SysID
            ModuleConfig(
                WHEEL_RADIUS,
                FREE_SPEED,
                WHEEL_COF,
                DCMotor.getKrakenX60Foc(1).withReduction(DRIVING_GEAR_RATIO),
                DRIVING_CURRENT_LIMIT,
                1
            ),
            *MODULE_POSITIONS.map { it.position.translation }.toTypedArray()
        )

        AutoBuilder.configure(
            this::estimatedPose,
            this::estimatedPose::set,
            this::measuredChassisSpeeds,
            this::desiredChassisSpeeds::set,
            PPHolonomicDriveController(
                PATH_FOLLOWING_TRANSLATION_GAINS.toPPLib(),
                PATH_FOLLOWING_ROTATION_GAINS.toPPLib()
            ),
            pathPlannerConfig,
            // Mirror path when the robot is on the red alliance (the robot starts on the opposite side of the field)
            {
                @Suppress("IDENTITY_SENSITIVE_OPERATIONS_WITH_VALUE_TYPE")
                DriverStation.getAlliance() == Optional.of(DriverStation.Alliance.Red)
            },
            this
        )

        if (Robot.model != Robot.Model.SIMULATION) {
            PathfindingCommand.warmupCommand().schedule()
        }
        if (io is DrivetrainIOSim) {
            io.registerPoseProviders(absolutePoseIOs.values.map { it.first })
        }

        PhoenixOdometryThread.getInstance().start()
    }

    override fun periodic() {
        if (Robot.model != Robot.Model.SIMULATION) {
            try {
                Robot.odometryLock.lock()
                io.updateInputs(inputs)
                Logger.processInputs("Drivetrain", inputs)
                val odometryTimestamps = io.getOdometryTimestamps()
                val odometryPositions = io.getOdometryPositions()
                val odometryYawPositons = io.getOdometryYawPositions()
                Logger.recordOutput("Drivetrain/Odometry Positions Count", odometryPositions[0].size)
                for (i in 0..<odometryTimestamps.size) {
                    val modulePositions = Array(4) { index ->
                        odometryPositions[index][i]
                    }
                    val moduleDeltas = Array(4) { index ->
                        SwerveModulePosition(
                            modulePositions[index].distanceMeters - lastModulePositions[index].distanceMeters,
                            modulePositions[index].angle - lastModulePositions[index].angle
                        )
                    }
                    for (moduleIndex in 0..3) {
                        lastModulePositions[moduleIndex] = modulePositions[moduleIndex]
                    }

                    rawGyroRotation = if (inputs.gyroConnected) {
                        Rotation2d.fromDegrees(odometryYawPositons[i])
                    } else {
                        rawGyroRotation.plus(Rotation2d(kinematics.toTwist2d(*moduleDeltas).dtheta))
                    }
                    poseEstimator.updateWithTime(odometryTimestamps[i], rawGyroRotation, modulePositions)
                }
            } finally {
                Robot.odometryLock.unlock()
            }
        } else {
            io.updateInputs(inputs)
            Logger.processInputs("Drivetrain", inputs)
            rawGyroRotation = inputs.gyroRotation
            poseEstimator.update(
                rawGyroRotation,
                inputs.measuredPositions.toTypedArray()
            )
        }


        // Update absolute pose sensors and add their measurements to the pose estimator
        for ((name, ioPair) in absolutePoseIOs) {
            val (sensorIO, inputs) = ioPair

            sensorIO.updateInputs(inputs)
            Logger.processInputs("Drivetrain/Absolute Pose/$name", inputs)

            Logger.recordOutput("Drivetrain/Absolute Pose/$name/Measurement Rejected", inputs.measurementRejected)
            if (inputs.measurement != null) {
                Logger.recordOutput("Drivetrain/Absolute Pose/$name/Measurement", inputs.measurement)
                Logger.recordOutput("Drivetrain/Absolute Pose/$name/Pose", inputs.measurement?.pose)
                if (!inputs.measurementRejected) {
                    inputs.measurement?.let {
                        poseEstimator.addAbsolutePoseMeasurement(it)
                        Logger.recordOutput("Drivetrain/Last Added Pose", it.pose)
                    }
                }
            }
        }

//        // Use the new measurements to update the pose estimator
//        poseEstimator.update(
//            inputs.gyroRotation,
//            inputs.measuredPositions.toTypedArray()
//        )

        Logger.recordOutput("Drivetrain/Pose Estimator/Estimated Pose", poseEstimator.estimatedPosition)
        Logger.recordOutput("Drivetrain/Chassis Speeds", measuredChassisSpeeds)
        Logger.recordOutput("Drivetrain/Desired Chassis Speeds", desiredChassisSpeeds)

        Logger.recordOutput(
            "Drivetrain/TagPoses", *FIELD_LAYOUT.tags
                .filter { tag ->
                    absolutePoseIOs.values.any { it.second.observedTags.contains(tag.ID) }
                }
                .map { it.pose }
                .toTypedArray())
    }

    /** The desired speeds and angles of the swerve modules. */
    private var desiredModuleStates
        get() = io.desiredStates
        set(value) {
            synchronized(this) {
                val stateArr = value.toTypedArray()
                SwerveDriveKinematics.desaturateWheelSpeeds(stateArr, FREE_SPEED)

                io.desiredStates = PerCorner.fromConventionalArray(stateArr)
                Logger.recordOutput("Drivetrain/Desired States", *stateArr)
            }
        }

    /**
     * The current speed of chassis relative to the ground,
     * assuming that the wheels have perfect traction with the ground.
     */
    val measuredChassisSpeeds get() = kinematics.cornerStatesToChassisSpeeds(inputs.measuredStates)

    /**
     * The chassis speeds that the drivetrain is attempting to move at.
     *
     * Note that the speeds are relative to the chassis, not the field.
     */
    private var desiredChassisSpeeds
        get() = kinematics.cornerStatesToChassisSpeeds(desiredModuleStates)
        set(value) {
            val discretized = ChassisSpeeds.discretize(value, Robot.period)
            desiredModuleStates = kinematics.toCornerSwerveModuleStates(discretized)
        }

    /** The estimated pose of the robot on the field, using the yaw value measured by the gyro. */
    var estimatedPose: Pose2d
        get() {
            return poseEstimator.estimatedPosition
        }
        private set(value) {
            poseEstimator.resetPosition(
                inputs.gyroRotation,
                inputs.measuredPositions.toTypedArray(),
                value
            )
        }

    val polarDrivingPIDController = PIDController(POLAR_DRIVING_GAINS).apply {
        enableContinuousInput(0.0, TAU)
    }

    fun getStatusSignals(): MutableList<BaseStatusSignal> {
        return io.getStatusSignals()
    }

    private fun isInDeadband(translation: Translation2d) =
        abs(translation.x) < JOYSTICK_DEADBAND && abs(translation.y) < JOYSTICK_DEADBAND

    private fun drive(translationInput: Translation2d, rotationInput: Translation2d) {
        if (isInDeadband(translationInput) && isInDeadband(rotationInput)) {
            // No joystick input - stop moving!
            desiredModuleStates = BRAKE_POSITION
        } else {
            desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                calculateInputCurve(translationInput.x) * FREE_SPEED.baseUnitMagnitude() * TRANSLATION_SENSITIVITY,
                calculateInputCurve(translationInput.y) * FREE_SPEED.baseUnitMagnitude() * TRANSLATION_SENSITIVITY,
                rotationInput.y * TAU * ROTATION_SENSITIVITY,
                estimatedPose.rotation
            )
        }
    }

    private fun driveWithoutDeadband(translationInput: Translation2d, rotationInput: Translation2d) {
        desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            calculateInputCurve(translationInput.x) * FREE_SPEED.baseUnitMagnitude() * TRANSLATION_SENSITIVITY,
            calculateInputCurve(translationInput.y) * FREE_SPEED.baseUnitMagnitude() * TRANSLATION_SENSITIVITY,
            rotationInput.y * TAU * ROTATION_SENSITIVITY,
            estimatedPose.rotation
        )
    }

    @Suppress("unused")
    fun driveWithJoystickPointingTowards(translationJoystick: Joystick, target: Translation2d): Command {
        Logger.recordOutput("Drivetrain/Polar Driving/Target", Pose2d(target, Rotation2d.kZero))
        Logger.recordOutput("Drivetrain/Polar Driving/Active", true)
        polarDrivingPIDController.setTolerance(0.5)
        polarDrivingPIDController.reset()
        return run {
            val translationInput = if (abs(translationJoystick.x) > JOYSTICK_DEADBAND
                || abs(translationJoystick.y) > JOYSTICK_DEADBAND
            ) {
                Translation2d(-translationJoystick.y, -translationJoystick.x)
            } else {
                Translation2d()
            }
            val magnitude = polarDrivingPIDController.calculate(
                target.minus(estimatedPose.translation).angle.radians - PI,
                estimatedPose.rotation.radians
            )

            Logger.recordOutput("Drivetrain/Polar Driving/PID Output", magnitude)

            desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                translationInput.x * FREE_SPEED.baseUnitMagnitude() * TRANSLATION_SENSITIVITY,
                translationInput.y * FREE_SPEED.baseUnitMagnitude() * TRANSLATION_SENSITIVITY,
                magnitude,
                inputs.gyroRotation
            )
        }.finallyDo { ->
            Logger.recordOutput("Drivetrain/Polar Driving/Active", false)
        }
    }

    private fun calculateInputCurve(input: Double): Double {
        val exponent = 1.7

        return input.absoluteValue.pow(exponent).withSign(input)
    }

    fun driveWithJoysticks(translationJoystick: Joystick, rotationJoystick: Joystick): Command =
        run {
            // Directly accessing Joystick.x/y gives inverted values - use a `Translation2d` instead.
            drive(translationJoystick.fieldRelativeTranslation2d, rotationJoystick.translation2d)
        }

    @Suppress("unused")
    fun driveWithController(controller: CommandXboxController): Command =
        run {
            val translationInput = Translation2d(controller.leftX, controller.leftY)
            val rotationInput = Translation2d(controller.rightX, controller.rightY)

            drive(translationInput, rotationInput)
        }

    fun zeroGyro(isReversed: Boolean = false, offset: Rotation2d = Rotation2d.kZero) {
        // Tell the gyro that the robot is facing the other alliance.
        var zeroPos = when (DriverStation.getAlliance().getOrNull()) {
            DriverStation.Alliance.Red -> Rotation2d.k180deg
            else -> Rotation2d.kZero
        }

        if (isReversed) {
            zeroPos += Rotation2d.k180deg
        }

        estimatedPose = Pose2d(estimatedPose.translation, zeroPos + offset)
//        io.setGyro(zeroPos)
    }

    fun zeroFull() {
        poseEstimator.resetPose(Pose2d(0.0, 0.0, Rotation2d.kZero))
    }

    var sysID = SysIdRoutine(
        SysIdRoutine.Config(
            0.5.voltsPerSecond, 2.volts, null
        ) {
            SignalLogger.writeString("state", it.toString())
        }, SysIdRoutine.Mechanism(
            io::runCharacterization,
            null,
            this,
        )
    )

    fun sysIdQuasistatic(direction: SysIdRoutine.Direction) = run {
        io.runCharacterization(0.volts, shouldStraight = true)
    }.withTimeout(2.0).andThen(sysID.quasistatic(direction))!!

    fun sysIdDynamic(direction: SysIdRoutine.Direction) = run {
        io.runCharacterization(0.volts, shouldStraight = true)
    }.withTimeout(2.0).andThen(sysID.dynamic(direction))!!

    fun sysIdQuasistaticSpin(direction: SysIdRoutine.Direction) = run {
        io.runCharacterization(0.volts, shouldSpin = true)
    }.withTimeout(2.0).andThen(sysID.quasistatic(direction))!!

    fun sysIdDynamicSpin(direction: SysIdRoutine.Direction) = run {
        io.runCharacterization(0.volts, shouldSpin = true)
    }.withTimeout(2.0).andThen(sysID.dynamic(direction))!!

    object Constants {
        // Translation/rotation coefficient for teleoperated driver controls
        /** Unit: Percent of max robot speed */
        const val TRANSLATION_SENSITIVITY = 1.0 // FIXME: Increase

        /** Unit: Rotations per second */
        const val ROTATION_SENSITIVITY = 0.8

        val ROBOT_LENGTH = 25.5.inches
        val ROBOT_WIDTH = 25.5.inches
        val TRACK_WIDTH = abs(TunerConstants.FrontLeft!!.LocationY - TunerConstants.FrontRight!!.LocationY)
        val WHEEL_BASE = abs(TunerConstants.FrontRight!!.LocationX - TunerConstants.BackRight!!.LocationX)
        val WHEEL_COF = 1.8 // FIXME: figure this out man idk

        val BUMPER_WIDTH = 30.inches
        val BUMPER_LENGTH = 30.inches

        const val JOYSTICK_DEADBAND = 0.075

        val FRONT_LEFT_CONSTANTS = TunerConstants.FrontLeft!!
        val FRONT_RIGHT_CONSTANTS = TunerConstants.FrontRight!!
        val BACK_RIGHT_CONSTANTS = TunerConstants.BackRight!!
        val BACK_LEFT_CONSTANTS = TunerConstants.BackLeft!!

        val FRONT_LEFT_MAGNET_OFFSET = FRONT_LEFT_CONSTANTS.EncoderOffset
        val FRONT_RIGHT_MAGNET_OFFSET = FRONT_RIGHT_CONSTANTS.EncoderOffset
        val BACK_RIGHT_MAGNET_OFFSET = BACK_RIGHT_CONSTANTS.EncoderOffset
        val BACK_LEFT_MAGNET_OFFSET = BACK_LEFT_CONSTANTS.EncoderOffset

        val MODULE_POSITIONS = PerCorner(
            frontLeft = Corner(
                Pose2d(
                    Translation2d(FRONT_LEFT_CONSTANTS.LocationX, FRONT_LEFT_CONSTANTS.LocationY),
                    Rotation2d.fromDegrees(0.0)
                ), FRONT_LEFT_MAGNET_OFFSET
            ),
            frontRight = Corner(
                Pose2d(
                    Translation2d(FRONT_RIGHT_CONSTANTS.LocationX, FRONT_RIGHT_CONSTANTS.LocationY),
                    Rotation2d.fromDegrees(180.0)
                ), FRONT_RIGHT_MAGNET_OFFSET
            ),
            backLeft = Corner(
                Pose2d(
                    Translation2d(BACK_LEFT_CONSTANTS.LocationX, BACK_LEFT_CONSTANTS.LocationY),
                    Rotation2d.fromDegrees(0.0)
                ), BACK_LEFT_MAGNET_OFFSET
            ),
            backRight = Corner(
                Pose2d(
                    Translation2d(BACK_RIGHT_CONSTANTS.LocationX, BACK_RIGHT_CONSTANTS.LocationY),
                    Rotation2d.fromDegrees(180.0)
                ), BACK_RIGHT_MAGNET_OFFSET
            ),
        )

        val DRIVE_BASE_RADIUS = hypot(MODULE_POSITIONS.frontLeft.position.x, MODULE_POSITIONS.frontLeft.position.y)

        // Chassis Control
        val FREE_SPEED = TunerConstants.kSpeedAt12Volts

        val PATH_FOLLOWING_TRANSLATION_GAINS = PIDGains(5.0)
        val PATH_FOLLOWING_ROTATION_GAINS = PIDGains(5.0)

        val POLAR_DRIVING_GAINS = PIDGains(0.15, 0.0, 0.05)

        // CAN IDs
        val MODULE_CAN_IDS =
            PerCorner(
                frontLeft =
                    Triple(
                        CTREDeviceId.FrontLeftDrivingMotor,
                        CTREDeviceId.FrontLeftTurningMotor,
                        CTREDeviceId.FrontLeftTurningEncoder
                    ),
                frontRight =
                    Triple(
                        CTREDeviceId.FrontRightDrivingMotor,
                        CTREDeviceId.FrontRightTurningMotor,
                        CTREDeviceId.FrontRightTurningEncoder
                    ),
                backLeft =
                    Triple(
                        CTREDeviceId.BackLeftDrivingMotor,
                        CTREDeviceId.BackLeftTurningMotor,
                        CTREDeviceId.BackLeftTurningEncoder
                    ),
                backRight =
                    Triple(
                        CTREDeviceId.BackRightDrivingMotor,
                        CTREDeviceId.BackRightTurningMotor,
                        CTREDeviceId.BackRightTurningEncoder
                    ),
            )

        /** A position with the modules radiating outwards from the center of the robot, preventing movement. */
        val BRAKE_POSITION =
            MODULE_POSITIONS.map { module -> SwerveModuleState(0.0, module.position.translation.angle) }
    }
}
