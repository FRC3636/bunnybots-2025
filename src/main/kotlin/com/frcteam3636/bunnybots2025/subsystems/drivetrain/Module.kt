package com.frcteam3636.bunnybots2025.subsystems.drivetrain

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.bunnybots2025.CANcoder
import com.frcteam3636.bunnybots2025.CTREDeviceId
import com.frcteam3636.bunnybots2025.TalonFX
import com.frcteam3636.bunnybots2025.utils.math.*
import com.frcteam3636.bunnybots2025.utils.swerve.speed
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Voltage
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation
import org.ironmaple.simulation.motorsims.SimulatedMotorController
import java.util.*

interface SwerveModule {
    // The current "state" of the swerve module.
    //
    // This is essentially the velocity of the wheel,
    // and includes both the speed and the angle
    // in which the module is currently traveling.
    val state: SwerveModuleState

    // The desired state of the module.
    //
    // This is the wheel velocity that we're trying to get to.
    var desiredState: SwerveModuleState

    // The measured position of the module.
    //
    // This is a vector with direction equal to the current angle of the module,
    // and magnitude equal to the total signed distance traveled by the wheel.
    val position: SwerveModulePosition

    var odometryTimestamps: DoubleArray
    var odometryTurnPositions: Array<Rotation2d>
    var odometryDrivePositionsRad: DoubleArray
    var odometryPositions: Array<SwerveModulePosition>

    fun getSignals(): Array<BaseStatusSignal> {
        return arrayOf()
    }

    fun periodic() {}
    fun characterize(voltage: Voltage)
}

class Mk5nSwerveModule(
    val drivingMotor: SwerveDrivingMotor, val turningMotor: SwerveTurningMotor, private val chassisAngle: Rotation2d
) : SwerveModule {
    private var timestampQueue: Queue<Double> = PhoenixOdometryThread.getInstance().makeTimestampQueue()

    override var odometryTimestamps: DoubleArray = doubleArrayOf()
    override var odometryDrivePositionsRad = doubleArrayOf()
    override var odometryTurnPositions: Array<Rotation2d> = emptyArray()
    override var odometryPositions: Array<SwerveModulePosition> = emptyArray()

    override val state: SwerveModuleState
        get() = SwerveModuleState(
            drivingMotor.velocity.inMetersPerSecond(),
            Rotation2d.fromRadians(turningMotor.position.inRadians()) + chassisAngle
        )

    override val position: SwerveModulePosition
        get() = SwerveModulePosition(
            drivingMotor.position, Rotation2d.fromRadians(turningMotor.position.inRadians()) + chassisAngle
        )

    override fun characterize(voltage: Voltage) {
        drivingMotor.setVoltage(voltage)
        turningMotor.position = -chassisAngle.measure
    }

    override var desiredState: SwerveModuleState = SwerveModuleState(0.0, -chassisAngle)
        get() = SwerveModuleState(field.speedMetersPerSecond, field.angle + chassisAngle)
        set(value) {
            val corrected = SwerveModuleState(value.speedMetersPerSecond, value.angle - chassisAngle)
            // optimize the state to avoid rotating more than 90 degrees
            corrected.optimize(
                Rotation2d.fromRadians(turningMotor.position.inRadians())
            )

            drivingMotor.velocity = corrected.speed
            turningMotor.position = corrected.angle.measure


            field = corrected
        }

    override fun getSignals(): Array<BaseStatusSignal> {
        return turningMotor.getSignals() + drivingMotor.getSignals()
    }

    override fun periodic() {
        odometryTimestamps = timestampQueue.stream().mapToDouble { it.toDouble() }.toArray()
        drivingMotor.periodic()
        turningMotor.periodic()
        odometryTurnPositions = turningMotor.odometryTurnPositions
        odometryDrivePositionsRad = drivingMotor.odometryDrivePositionsRad
        odometryPositions = Array(odometryTimestamps.size) { index ->
            SwerveModulePosition(
                odometryDrivePositionsRad[index] * WHEEL_RADIUS.inInches(),
                odometryTurnPositions[index]
            )
        }
        timestampQueue.clear()
    }
}

interface SwerveTurningMotor {
    var position: Angle
    var odometryTurnPositions: Array<Rotation2d>
    fun getSignals(): Array<BaseStatusSignal> {
        return arrayOf()
    }
    fun periodic() {}
}

interface SwerveDrivingMotor {
    val position: Distance
    var velocity: LinearVelocity
    var odometryDrivePositionsRad: DoubleArray
    fun setVoltage(voltage: Voltage)
    fun getSignals(): Array<BaseStatusSignal> {
        return arrayOf()
    }
    fun periodic() {}
}

class DrivingTalon(id: CTREDeviceId) : SwerveDrivingMotor {

    private val inner = TalonFX(id).apply {
        configurator.apply(TalonFXConfiguration().apply {
            Slot0.apply {
                pidGains = DRIVING_PID_GAINS_TALON
                motorFFGains = DRIVING_FF_GAINS_TALON
            }
//            CurrentLimits.apply {
//                SupplyCurrentLimit = DRIVING_CURRENT_LIMIT.inAmps()
//                SupplyCurrentLimitEnable = true
//            }
            Feedback.apply {
                SensorToMechanismRatio = DRIVING_GEAR_RATIO
            }
        })
    }

    override var odometryDrivePositionsRad = doubleArrayOf()

    private val positionSignal = inner.position
    private val velocitySignal = inner.velocity

    private val positionQueue: Queue<Double> = PhoenixOdometryThread.getInstance().registerSignal(positionSignal.clone())

    init {
        BaseStatusSignal.setUpdateFrequencyForAll(250.0, positionSignal, velocitySignal)
        inner.optimizeBusUtilization()
    }

    override val position: Distance
        get() = positionSignal.value.toLinear(WHEEL_RADIUS)

    private var velocityControl = VelocityVoltage(0.0).apply {
        EnableFOC = true
    }

    override var velocity: LinearVelocity
        get() = velocitySignal.value.toLinear(WHEEL_RADIUS)
        set(value) {
            inner.setControl(velocityControl.withVelocity(value.toAngular(WHEEL_RADIUS)))
        }

    private val voltageControl = VoltageOut(0.0).apply {
        EnableFOC = true
    }

    override fun setVoltage(voltage: Voltage) {
        inner.setControl(voltageControl.withOutput(voltage.inVolts()))
    }

    override fun getSignals(): Array<BaseStatusSignal> {
        return arrayOf(positionSignal, velocitySignal)
    }

    override fun periodic() {
        odometryDrivePositionsRad = positionQueue.stream().mapToDouble { Units.rotationsToRadians(it) }.toArray()
        positionQueue.clear()
    }
}

class TurningTalon(id: CTREDeviceId, encoderId: CTREDeviceId, magnetOffset: Double) : SwerveTurningMotor {

    private val inner = TalonFX(id).apply {
        configurator.apply(TalonFXConfiguration().apply {
            Slot0.apply {
                pidGains = TURNING_PID_GAINS
                motorFFGains = TURNING_FF_GAINS
                MotorOutput.apply {
                    NeutralMode = NeutralModeValue.Brake
                }
                Feedback.apply {
                    FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder
                    RotorToSensorRatio = TURNING_GEAR_RATIO
                    FeedbackRemoteSensorID = encoderId.num
                }
//                CurrentLimits.apply {
//                    StatorCurrentLimit = TURNING_CURRENT_LIMIT.inAmps()
//                    StatorCurrentLimitEnable = true
//                }
            }
        })
    }

    private val positionSignal = inner.position

    private val positionQueue: Queue<Double> = PhoenixOdometryThread.getInstance().registerSignal(positionSignal.clone())
    override var odometryTurnPositions: Array<Rotation2d> = emptyArray()

    init {
        CANcoder(encoderId).apply {
            configurator.apply(CANcoderConfiguration().apply {
                MagnetSensor.MagnetOffset = magnetOffset
            })
        }
        BaseStatusSignal.setUpdateFrequencyForAll(250.0, positionSignal)
        inner.optimizeBusUtilization()
    }

    private val positonControl = PositionVoltage(0.0).apply {
        EnableFOC = true
    }

    override var position: Angle
        set(value) {
            inner.setControl(positonControl.withPosition(value))
        }
        get() = positionSignal.value

    override fun getSignals(): Array<BaseStatusSignal> {
        return arrayOf(positionSignal)
    }

    override fun periodic() {
        odometryTurnPositions = positionQueue.map { Rotation2d.fromRotations(it) }.toTypedArray()
        positionQueue.clear()
    }
}

class SimSwerveModule(val sim: SwerveModuleSimulation) : SwerveModule {

    override var odometryDrivePositionsRad: DoubleArray = doubleArrayOf()
    override var odometryTurnPositions: Array<Rotation2d> = emptyArray()
    override var odometryPositions: Array<SwerveModulePosition> = emptyArray()
    private val driveMotor: SimulatedMotorController.GenericMotorController = sim.useGenericMotorControllerForDrive()
//        .withCurrentLimit(DRIVING_CURRENT_LIMIT)

    // reference to the simulated turn motor
    private val turnMotor: SimulatedMotorController.GenericMotorController = sim.useGenericControllerForSteer()
//        .withCurrentLimit(TURNING_CURRENT_LIMIT)

    // TODO: figure out what the moment of inertia actually is and if it even matters
    private val drivingFeedforward = SimpleMotorFeedforward(DRIVING_FF_GAINS_TALON)
    private val drivingFeedback = PIDController(DRIVING_PID_GAINS_TALON)

    private val turningFeedback = PIDController(TURNING_PID_GAINS).apply { enableContinuousInput(0.0, TAU) }

    override var odometryTimestamps: DoubleArray = doubleArrayOf()

    override val state: SwerveModuleState
        get() = SwerveModuleState(
            sim.driveWheelFinalSpeed.inRadiansPerSecond() * WHEEL_RADIUS.inMeters(),
            sim.steerAbsoluteFacing
        )

    override var desiredState: SwerveModuleState = SwerveModuleState(0.0, Rotation2d())
        set(value) {
            field = value.apply {
                optimize(state.angle)
            }
        }

    override val position: SwerveModulePosition
        get() = SwerveModulePosition(
            sim.driveWheelFinalPosition.toLinear(WHEEL_RADIUS), sim.steerAbsoluteFacing
        )

    override fun periodic() {
        // Set the new input voltages
        turnMotor.requestVoltage(
            Volts.of(turningFeedback.calculate(state.angle.radians, desiredState.angle.radians))
        )
        driveMotor.requestVoltage(
            Volts.of(
                drivingFeedforward.calculate(desiredState.speedMetersPerSecond) + drivingFeedback.calculate(
                    state.speedMetersPerSecond, desiredState.speedMetersPerSecond
                )
            )
        )
    }

    override fun characterize(voltage: Voltage) {
        TODO("Not yet implemented")
    }
}

// take the known wheel diameter, divide it by two to get the radius, then get the
// circumference
internal val WHEEL_RADIUS = 2.inches

const val DRIVING_GEAR_RATIO = TunerConstants.kDriveGearRatio
const val TURNING_GEAR_RATIO = TunerConstants.kSteerGearRatio

internal val DRIVING_PID_GAINS_TALON: PIDGains = TunerConstants.driveGains!!.pidGains
internal val DRIVING_FF_GAINS_TALON: MotorFFGains = TunerConstants.driveGains!!.motorFFGains

internal val TURNING_PID_GAINS: PIDGains = TunerConstants.steerGains!!.pidGains
internal val TURNING_FF_GAINS: MotorFFGains = TunerConstants.steerGains!!.motorFFGains