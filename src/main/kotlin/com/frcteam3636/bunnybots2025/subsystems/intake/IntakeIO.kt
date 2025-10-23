package com.frcteam3636.bunnybots2025.subsystems.intake

import com.frcteam3636.bunnybots2025.REVMotorControllerId
import com.frcteam3636.bunnybots2025.SparkFlex
import com.frcteam3636.bunnybots2025.utils.math.*
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkBase.PersistMode
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkFlexConfig
import edu.wpi.first.units.Units.*
import org.team9432.annotation.Logged

@Logged
open class IntakeInputs {
    var rollerVelocity = RadiansPerSecond.zero()
    var current = Amps.zero()!!
    var position = Radians.zero()
}

interface IntakeIO {
    fun setSpeed(percentage: Double)
    fun updateInputs(inputs: IntakeInputs)
}

class IntakeIOReal: IntakeIO {

    private var intakeMotor = SparkFlex(REVMotorControllerId.IntakeMotor, SparkLowLevel.MotorType.kBrushless)
    private var intakePivotMotor = SparkFlex(REVMotorControllerId.IntakePivotMotor, SparkLowLevel.MotorType.kBrushless)

    override fun setSpeed(percentage: Double) {
        assert(percentage in -1.0 .. 1.0)
        intakeMotor.set(percentage)
    }

    override fun updateInputs(inputs: IntakeInputs) {
        inputs.rollerVelocity = intakeMotor.encoder.velocity.rpm
        inputs.current = intakeMotor.outputCurrent.amps
        inputs.position = intakePivotMotor.encoder.position.rotations
    }

    init {
        val config = SparkFlexConfig().apply {
            idleMode(SparkBaseConfig.IdleMode.kBrake)
            PIDController(PID_GAINS)
        }

        intakePivotMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }

    internal companion object Constants {
        val PID_GAINS = PIDGains(6.0, 0.0, 0.0)
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