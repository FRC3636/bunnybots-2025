package com.frcteam3636.bunnybots2025.subsystems.shooter

import com.frcteam3636.bunnybots2025.REVMotorControllerId
import com.frcteam3636.bunnybots2025.SparkFlex
import com.frcteam3636.bunnybots2025.utils.math.*
import com.revrobotics.spark.SparkLowLevel
import edu.wpi.first.units.Units.*
import org.team9432.annotation.Logged

@Logged
open class FlywheelInputs {
    var topVelocity = RotationsPerSecond.zero()!!
    var topCurrent = Amps.zero()!!
    var bottomVelocity = RotationsPerSecond.zero()!!
    var bottomCurrent = Amps.zero()!!
}

interface FlywheelIO {
    fun setSpeed(upperPercent: Double, lowerPercent: Double)
    fun setVoltage(upperVoltage: Double, lowerVoltage: Double)
    fun updateInputs(inputs: FlywheelInputs)
}

class FlywheelIOReal : FlywheelIO {

    private val upperShooterMotor = SparkFlex(REVMotorControllerId.UpperShooterMotor, SparkLowLevel.MotorType.kBrushless)
    private val lowerShooterMotor = SparkFlex(REVMotorControllerId.LowerShooterMotor, SparkLowLevel.MotorType.kBrushless)

    override fun setSpeed(upperPercent: Double, lowerPercent: Double) {
        assert(upperPercent in -1.0..1.0)
        assert(lowerPercent in -1.0..1.0)
        upperShooterMotor.set(upperPercent)
        lowerShooterMotor.set(lowerPercent)
    }

    override fun setVoltage(upperVoltage: Double, lowerVoltage: Double) {
        assert(upperVoltage in -13.0..13.0)
        assert(lowerVoltage in -13.0..13.0)
        upperShooterMotor.setVoltage(upperVoltage)
        lowerShooterMotor.setVoltage(lowerVoltage)
    }

    override fun updateInputs(inputs: FlywheelInputs) {
        inputs.topVelocity = upperShooterMotor.encoder.velocity.rpm
        inputs.topCurrent = upperShooterMotor.outputCurrent.amps
        inputs.bottomVelocity = lowerShooterMotor.encoder.velocity.rpm
        inputs.bottomCurrent = lowerShooterMotor.outputCurrent.amps
    }
}

class FlywheelIOSim: FlywheelIO {
    override fun setSpeed(upperPercent: Double, lowerPercent: Double) {
        TODO("Not yet implemented")
    }

    override fun updateInputs(inputs: FlywheelInputs) {
        TODO("Not yet implemented")
    }

    override fun setVoltage(upperVoltage: Double, lowerVoltage: Double) {
        TODO("Not yet implemented")
    }

}