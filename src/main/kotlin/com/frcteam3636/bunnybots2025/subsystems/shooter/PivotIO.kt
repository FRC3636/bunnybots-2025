package com.frcteam3636.bunnybots2025.subsystems.shooter

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.bunnybots2025.CTREDeviceId
import com.frcteam3636.bunnybots2025.TalonFX
import com.frcteam3636.bunnybots2025.subsystems.intake.IntakeIOReal.Constants.ACCELERATION
import com.frcteam3636.bunnybots2025.subsystems.intake.IntakeIOReal.Constants.CRUISE_VELOCITY
import com.frcteam3636.bunnybots2025.utils.math.PIDGains
import com.frcteam3636.bunnybots2025.utils.math.degrees
import com.frcteam3636.bunnybots2025.utils.math.degreesPerSecond
import com.frcteam3636.bunnybots2025.utils.math.inRotationsPerSecond
import com.frcteam3636.bunnybots2025.utils.math.inRotationsPerSecondPerSecond
import com.frcteam3636.bunnybots2025.utils.math.pidGains
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.Alert
import org.team9432.annotation.Logged

@Logged
open class PivotInputs {
    var pivotAngle = Rotations.zero()!!
    var pivotCurrent = Amps.zero()!!
    var pivotVelocity = RotationsPerSecond.zero()!!
    var pivotMotorTemperature = Celsius.zero()!!
    var pivotDisabled = false
}

interface PivotIO {
    fun turnToAngle(angle: Angle)
    fun setBrakeMode(enabled: Boolean) {}
    fun disablePivot() {}
    fun getStatusSignals(): MutableList<BaseStatusSignal> {
        return mutableListOf()
    }

    fun updateInputs(inputs: PivotInputs)
}

class PivotIOReal : PivotIO {
    private var pivotDisabled = false
    private var brakeModeEnabled = true

    private val shooterPivotMotor = TalonFX(CTREDeviceId.ShooterPivotMotor).apply {
        configurator.apply(TalonFXConfiguration().apply {
            Slot0.apply {
                pidGains = PID_GAINS
                MotionMagic.apply {
                    MotionMagicCruiseVelocity = CRUISE_VELOCITY.inRotationsPerSecond()
                    MotionMagicAcceleration = ACCELERATION.inRotationsPerSecondPerSecond()
                }
            }
            MotionMagic.apply {
                MotionMagicCruiseVelocity = PROFILE_VELOCITY
                MotionMagicAcceleration = PROFILE_ACCELERATION
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
    }

    val positionControl = MotionMagicVoltage(0.0)

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

    override fun getStatusSignals(): MutableList<BaseStatusSignal> {
        return mutableListOf(positionSignal, currentSignal, velocitySignal)
    }

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

    internal companion object Constants {
        private val PID_GAINS = PIDGains(6.0, 0.0, 0.0)
        private const val SENSOR_TO_MECHANISM_GEAR_RATIO = 0.0
        private const val ROTOR_TO_SENSOR_GEAR_RATIO = 0.0
        private const val PROFILE_ACCELERATION = 50.0
        private const val PROFILE_JERK = 0.0
        private const val PROFILE_VELOCITY = 25.0
    }
}

class PivotIOSim : PivotIO {
    override fun turnToAngle(angle: Angle) {
        TODO("Not yet implemented")
    }

    override fun updateInputs(inputs: PivotInputs) {
        TODO("Not yet implemented")
    }

}

