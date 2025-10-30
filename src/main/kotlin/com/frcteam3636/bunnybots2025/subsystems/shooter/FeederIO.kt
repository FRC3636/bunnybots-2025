package com.frcteam3636.bunnybots2025.subsystems.shooter

import com.frcteam3636.bunnybots2025.REVMotorControllerId
import com.frcteam3636.bunnybots2025.SparkFlex
import com.frcteam3636.bunnybots2025.utils.math.amps
import com.frcteam3636.bunnybots2025.utils.math.celsius
import com.frcteam3636.bunnybots2025.utils.math.rpm
import com.revrobotics.spark.SparkLowLevel
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Celsius
import edu.wpi.first.units.Units.RotationsPerSecond
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
    private val shooterFeederMotor =
        SparkFlex(REVMotorControllerId.ShooterFeederMotor, SparkLowLevel.MotorType.kBrushless)

    override fun setSpeed(percent: Double) {
        assert(percent in -1.0..1.0)
        shooterFeederMotor.set(percent)
    }

    override fun updateInputs(inputs: FeederInputs) {
        inputs.feederVelocity = shooterFeederMotor.encoder.velocity.rpm
        inputs.feederCurrent = shooterFeederMotor.outputCurrent.amps
        inputs.feederTemperature = shooterFeederMotor.motorTemperature.celsius
    }
}

class FeederIOSim : FeederIO {
    override fun setSpeed(percent: Double) {
        TODO("Not yet implemented")
    }

    override fun updateInputs(inputs: FeederInputs) {
        TODO("Not yet implemented")
    }
}