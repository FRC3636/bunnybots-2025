package com.frcteam3636.bunnybots2025.subsystems.intake

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.bunnybots2025.CTREDeviceId
import com.frcteam3636.bunnybots2025.REVMotorControllerId
import com.frcteam3636.bunnybots2025.SparkFlex
import com.frcteam3636.bunnybots2025.TalonFX
import com.frcteam3636.bunnybots2025.subsystems.drivetrain.TURNING_PID_GAINS
import com.frcteam3636.bunnybots2025.utils.math.*
import com.revrobotics.spark.SparkLowLevel
import edu.wpi.first.units.Units.*
import org.team9432.annotation.Logged

@Logged
open class IntakeInputs {
    var rollerVelocity = RadiansPerSecond.zero()!!
    var rollerCurrent = Amps.zero()!!
    var pivotPosition = Radians.zero()!!
    var pivotCurrent = Amps.zero()!!
}

interface IntakeIO {
    fun setSpeed(percentage: Double)
    fun updateInputs(inputs: IntakeInputs)
    fun getSignals(): MutableList<BaseStatusSignal> {
        return mutableListOf()
    }
}

class IntakeIOReal: IntakeIO {

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
                SensorToMechanismRatio = GEAR_RATIO
            }
            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Brake
            }
        })
    }

    private val positionSignal = intakePivotMotor.position
    private val currentSignal = intakePivotMotor.supplyCurrent

    override fun setSpeed(percentage: Double) {
        assert(percentage in -1.0 .. 1.0)
        intakeMotor.set(percentage)
    }

    override fun updateInputs(inputs: IntakeInputs) {
        inputs.rollerVelocity = intakeMotor.encoder.velocity.rpm
        inputs.rollerCurrent = intakeMotor.outputCurrent.amps
        inputs.pivotPosition = positionSignal.valueAsDouble.radians
        inputs.pivotCurrent = currentSignal.valueAsDouble.amps
    }

    override fun getSignals(): MutableList<BaseStatusSignal> {
        return mutableListOf(positionSignal, currentSignal)
    }

    internal companion object Constants {
        val PID_GAINS = PIDGains(6.0, 0.0, 0.0)
        val CRUISE_VELOCITY = 0.0.rotationsPerSecond
        val ACCELERATION = 0.0.rotationsPerSecondPerSecond
        const val GEAR_RATIO = 0.0
    }
}

class IntakeIOSim: IntakeIO {
    override fun setSpeed(percentage: Double) {
        TODO("Not yet implemented")
    }

    override fun updateInputs(inputs: IntakeInputs) {
        TODO("Not yet implemented")
    }

}