package com.frcteam3636.bunnybots2025.subsystems.shooter

import com.frcteam3636.bunnybots2025.REVDeviceId
import com.frcteam3636.bunnybots2025.SparkFlex
import com.frcteam3636.bunnybots2025.utils.math.amps
import com.frcteam3636.bunnybots2025.utils.math.celsius
import com.frcteam3636.bunnybots2025.utils.math.rpm
import com.revrobotics.spark.SparkLowLevel
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import org.team9432.annotation.Logged

@Logged
open class FeederInputs {
    var feederVelocity = RotationsPerSecond.zero()!!
    var feederCurrent = Amps.zero()!!
    var feederTemperature = Celsius.zero()!!
}

interface FeederIO {
    fun setSpeed(percent: Double)
    fun updateInputs(inputs: FeederInputs)
}

class FeederIOReal : FeederIO {
    private val feederMotor =
        SparkFlex(REVDeviceId.ShooterFeederMotor, SparkLowLevel.MotorType.kBrushless)

    override fun setSpeed(percent: Double) {
        assert(percent in -1.0..1.0)
        feederMotor.set(percent)
    }

    override fun updateInputs(inputs: FeederInputs) {
        inputs.feederVelocity = feederMotor.encoder.velocity.rpm
        inputs.feederCurrent = feederMotor.outputCurrent.amps
        inputs.feederTemperature = feederMotor.motorTemperature.celsius
    }
}

class FeederIOSim : FeederIO {
    private val feederMotorSystem = LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.0001, 1.0)
    private val feederMotor = DCMotorSim(feederMotorSystem, DCMotor.getNeoVortex(1))

    override fun setSpeed(percent: Double) {
        feederMotor.inputVoltage = percent * 12.0
    }

    override fun updateInputs(inputs: FeederInputs) {
        inputs.feederVelocity = feederMotor.angularVelocity
        inputs.feederCurrent = feederMotor.currentDrawAmps.amps
    }
}