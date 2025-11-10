package com.frcteam3636.bunnybots2025.subsystems.shooter

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.bunnybots2025.CANcoder
import com.frcteam3636.bunnybots2025.CTREDeviceId
import com.frcteam3636.bunnybots2025.TalonFX
import com.frcteam3636.bunnybots2025.subsystems.intake.IntakeIOReal.Constants.PROFILE_CRUISE_VELOCITY
import com.frcteam3636.bunnybots2025.utils.math.*
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.Timer
import org.team9432.annotation.Logged

@Logged
open class PivotInputs {
    var pivotAngle = 10.degrees // we don't initialize to 0 to avoid accidentally disabling the pivot on startup
    var pivotCurrent = Amps.zero()!!
    var pivotVelocity = RotationsPerSecond.zero()!!
    var pivotMotorTemperature = Celsius.zero()!!
    var pivotDisabled = false
}

interface PivotIO {
    fun turnToAngle(angle: Angle)
    fun setBrakeMode(enabled: Boolean) {}
    fun disablePivot() {}

    val signals: Array<BaseStatusSignal>
        get() = emptyArray()

    fun updateInputs(inputs: PivotInputs)
}

class PivotIOReal : PivotIO {
    private var pivotDisabled = false
    private var brakeModeEnabled = true

    private val shooterPivotMotor = TalonFX(CTREDeviceId.ShooterPivotMotor).apply {
        configurator.apply(TalonFXConfiguration().apply {
            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Brake
            }
            Slot0.apply {
                pidGains = PID_GAINS
            }
            MotionMagic.apply {
                MotionMagicCruiseVelocity = PROFILE_CRUISE_VELOCITY.inRotationsPerSecond()
                MotionMagicAcceleration = PROFILE_ACCELERATION.inRotationsPerSecondPerSecond()
                MotionMagicJerk = PROFILE_JERK
            }
            Feedback.apply {
                FeedbackRemoteSensorID = CTREDeviceId.ShooterPivotEncoder.num
                FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder
                SensorToMechanismRatio = SENSOR_TO_MECHANISM_GEAR_RATIO
                RotorToSensorRatio = ROTOR_TO_SENSOR_GEAR_RATIO
            }
        })
    }

    private val positionSignal = shooterPivotMotor.position
    private val currentSignal = shooterPivotMotor.supplyCurrent
    private val velocitySignal = shooterPivotMotor.velocity
    private val temperatureSignal = shooterPivotMotor.deviceTemp

    init {
        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            positionSignal,
            currentSignal,
            velocitySignal,
            temperatureSignal
        )
        shooterPivotMotor.optimizeBusUtilization()
        CANcoder(CTREDeviceId.ShooterPivotEncoder).apply {
            configurator.apply(CANcoderConfiguration().apply {
                MagnetSensor.MagnetOffset = MAGNET_OFFSET - HARDSTOP_OFFSET
            })
        }
    }

    val positionControl = MotionMagicVoltage(0.0).withUpdateFreqHz(0.0)

    override fun turnToAngle(angle: Angle) {
        assert(angle > 0.degrees)
        assert(angle < 90.degrees) // TODO: Find out if we need to raise this
        if (pivotDisabled)
            return
        shooterPivotMotor.setControl(positionControl.withPosition(angle))
    }

    override fun setBrakeMode(enabled: Boolean) {
        brakeModeEnabled = enabled
        shooterPivotMotor.setNeutralMode(
            if (enabled) {
                NeutralModeValue.Brake
            } else {
                NeutralModeValue.Coast
            }
        )
    }

    override val signals: Array<BaseStatusSignal>
        get() = arrayOf(positionSignal, currentSignal, velocitySignal, temperatureSignal)

    override fun updateInputs(inputs: PivotInputs) {
        inputs.pivotAngle = positionSignal.value
        inputs.pivotCurrent = currentSignal.value
        inputs.pivotVelocity = velocitySignal.value
        inputs.pivotMotorTemperature = temperatureSignal.value
        inputs.pivotDisabled = pivotDisabled
    }

    override fun disablePivot() {
        pivotDisabled = true
        // this causes a sizeable loop overrun if we aren't in brake mode,
        // but I'm willing to do this to prevent the
        // robot from tearing itself apart
        if (!brakeModeEnabled)
            setBrakeMode(true)
        shooterPivotMotor.setControl(NeutralOut())
    }

    companion object Constants {
        private val PID_GAINS = PIDGains(6.0, 0.0, 0.0)
        private const val SENSOR_TO_MECHANISM_GEAR_RATIO = 0.0
        private const val ROTOR_TO_SENSOR_GEAR_RATIO = 0.0
        private const val MAGNET_OFFSET = 0.0
        private val HARDSTOP_OFFSET = 12.degrees.inRotations()
        val PROFILE_ACCELERATION = 5.0.degreesPerSecondPerSecond
        const val PROFILE_JERK = 0.0
        val PROFILE_VELOCITY = 5.0.degreesPerSecond
    }
}

class PivotIOSim : PivotIO {
    private val profile = TrapezoidProfile(
        TrapezoidProfile.Constraints(
            PivotIOReal.PROFILE_VELOCITY.inRotationsPerSecond(),
            PivotIOReal.PROFILE_ACCELERATION.inRotationsPerSecondPerSecond(),
        )
    )

    private val profileTimer = Timer().apply { start() }

    private var start = TrapezoidProfile.State()
    private var goal = TrapezoidProfile.State()

    override fun turnToAngle(angle: Angle) {
        start = profile.calculate(profileTimer.get(), start, goal)
        goal = TrapezoidProfile.State(angle.inRotations(), 0.0)
        profileTimer.reset()
    }

    override fun updateInputs(inputs: PivotInputs) {
        val state = profile.calculate(profileTimer.get(), start, goal)
        inputs.pivotVelocity = state.velocity.rotationsPerSecond
        inputs.pivotAngle = state.position.rotations
    }
}

