package com.frcteam3636.bunnybots2025.subsystems.shooter

import com.ctre.phoenix6.configs.CANrangeConfiguration
import com.ctre.phoenix6.hardware.CANrange
import com.ctre.phoenix6.signals.UpdateModeValue
import com.frcteam3636.bunnybots2025.CTREDeviceId
import com.frcteam3636.bunnybots2025.REVMotorControllerId
import com.frcteam3636.bunnybots2025.SparkFlex
import com.frcteam3636.bunnybots2025.utils.math.amps
import com.frcteam3636.bunnybots2025.utils.math.rpm
import com.revrobotics.spark.SparkLowLevel
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.RotationsPerSecond
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

@Logged
open class FeederInputs {
    var feederVelocity = RotationsPerSecond.zero()
    var feederCurrent = Amps.zero()
}

interface FeederIO {
    fun setSpeed(percent: Double)
    fun updateInputs(inputs: FeederInputs)
}

class FeederIOReal: FeederIO {
    private val shooterFeederMotor = SparkFlex(REVMotorControllerId.ShooterFeederMotor, SparkLowLevel.MotorType.kBrushless)

    private var canRange = CANrange(CTREDeviceId.CANRange.num).apply {
        configurator.apply(
            CANrangeConfiguration().apply {
                ProximityParams.ProximityThreshold = 0.35 // fix
                FovParams.FOVCenterY = 10.0 // fix
                FovParams.FOVRangeY = 7.0 // fix
                ToFParams.UpdateMode = UpdateModeValue.ShortRangeUserFreq
                ToFParams.UpdateFrequency = 50.0
            }
        )
    }

    override fun setSpeed(percent: Double) {
        assert(percent in -1.0 .. 1.0)
        shooterFeederMotor.set(percent)
    }

    override fun updateInputs(inputs: FeederInputs) {
        inputs.feederVelocity = shooterFeederMotor.encoder.velocity.rpm
        inputs.feederCurrent = shooterFeederMotor.outputCurrent.amps
    }
}

class FeederIOSim: FeederIO {
    override fun setSpeed(percent: Double) {
        TODO("Not yet implemented")
    }

    override fun updateInputs(inputs: FeederInputs) {
        TODO("Not yet implemented")
    }

}