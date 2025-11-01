package com.frcteam3636.bunnybots2025.subsystems.intake

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.bunnybots2025.*
import com.frcteam3636.bunnybots2025.utils.math.*
import com.revrobotics.spark.SparkLowLevel
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import org.team9432.annotation.Logged

@Logged
open class IntakeInputs {
    var rollerVelocity = RotationsPerSecond.zero()!!
    var rollerCurrent = Amps.zero()!!
    var rollerTemperature = Celsius.zero()!!
    var pivotPosition = Rotations.zero()!!
    var pivotCurrent = Amps.zero()!!
    var pivotVelocity = RotationsPerSecond.zero()!!
    var pivotTemperature = Celsius.zero()!!
    var pivotDisabled = false
}

interface IntakeIO {
    fun setRollerSpeed(percentage: Double)
    fun setPivotPosition(pivotPosition: Angle)
    fun updateInputs(inputs: IntakeInputs)
    fun disablePivot() {}
    fun setBrakeMode(enabled: Boolean) {}
    fun getSignals(): MutableList<BaseStatusSignal> {
        return mutableListOf()
    }
}

class IntakeIOReal : IntakeIO {
    private var pivotDisabled = false
    private var brakeModeEnabled = true

    private var intakeMotor = SparkFlex(REVMotorControllerId.IntakeMotor, SparkLowLevel.MotorType.kBrushless)
    private var intakePivotMotor = TalonFX(CTREDeviceId.IntakePivotMotor).apply {
        configurator.apply(TalonFXConfiguration().apply {
            Slot0.apply {
                pidGains = PID_GAINS
                MotionMagic.apply {
                    MotionMagicCruiseVelocity = CRUISE_VELOCITY.inRotationsPerSecond()
                    MotionMagicAcceleration = ACCELERATION.inRotationsPerSecondPerSecond()
                }
            }
            Feedback.apply {
                FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder
                FeedbackRemoteSensorID = CTREDeviceId.IntakePivotEncoder.num
                SensorToMechanismRatio = ENCODER_TO_PIVOT_GEAR_RATIO
                RotorToSensorRatio = MOTOR_TO_ENCODER_GEAR_RATIO
            }
            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Brake
            }
        })
    }

    init {
        CANcoder(CTREDeviceId.IntakePivotEncoder).apply {
            configurator.apply(CANcoderConfiguration().apply {
                MagnetSensor.apply {
                    MagnetOffset = ENCODER_MAGNET_OFFSET
                }
            })
        }
    }

    private val positionSignal = intakePivotMotor.position
    private val currentSignal = intakePivotMotor.supplyCurrent
    private val velocitySignal = intakePivotMotor.velocity
    private val temperatureSignal = intakePivotMotor.deviceTemp

    private val positionControl = MotionMagicVoltage(0.0.rotations)

    override fun setRollerSpeed(percentage: Double) {
        assert(percentage in -1.0..1.0)
        intakeMotor.set(percentage)
    }

    override fun setPivotPosition(pivotPosition: Angle) {
        if (pivotDisabled)
            return
        intakePivotMotor.setControl(positionControl.withPosition(pivotPosition))
    }

    override fun setBrakeMode(enabled: Boolean) {
        intakePivotMotor.setNeutralMode(
            if (enabled) {
                NeutralModeValue.Brake
            } else {
                NeutralModeValue.Coast
            }
        )
    }

    override fun updateInputs(inputs: IntakeInputs) {
        inputs.rollerVelocity = intakeMotor.encoder.velocity.rpm
        inputs.rollerCurrent = intakeMotor.outputCurrent.amps
        inputs.pivotPosition = positionSignal.value
        inputs.pivotCurrent = currentSignal.value
        inputs.rollerVelocity = velocitySignal.value
        inputs.pivotTemperature = temperatureSignal.value
        inputs.rollerTemperature = intakeMotor.motorTemperature.celsius
        inputs.pivotDisabled = pivotDisabled
    }

    override fun getSignals(): MutableList<BaseStatusSignal> {
        return mutableListOf(positionSignal, currentSignal, temperatureSignal)
    }

    override fun disablePivot() {
        pivotDisabled = true
        // this causes a sizeable loop overrun, but I'm willing to do this
        // to prevent the robot from tearing itself apart
        if (!brakeModeEnabled)
            setBrakeMode(true)
        intakePivotMotor.setControl(NeutralOut())
    }

    internal companion object Constants {
        val PID_GAINS = PIDGains(6.0, 0.0, 0.0)
        val CRUISE_VELOCITY = 0.0.rotationsPerSecond
        val ACCELERATION = 0.0.rotationsPerSecondPerSecond
        const val ENCODER_MAGNET_OFFSET = 0.0
        const val ENCODER_TO_PIVOT_GEAR_RATIO = 0.0
        const val MOTOR_TO_ENCODER_GEAR_RATIO = 0.0
    }
}

class IntakeIOSim : IntakeIO {
    override fun setRollerSpeed(percentage: Double) {
        TODO("Not yet implemented")
    }

    override fun updateInputs(inputs: IntakeInputs) {
        TODO("Not yet implemented")
    }

    override fun setPivotPosition(pivotPosition: Angle) {
        TODO("Not yet implemented")
    }
}