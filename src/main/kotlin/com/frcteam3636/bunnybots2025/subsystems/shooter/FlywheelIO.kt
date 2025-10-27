package com.frcteam3636.bunnybots2025.subsystems.shooter

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.bunnybots2025.CTREDeviceId
import com.frcteam3636.bunnybots2025.REVMotorControllerId
import com.frcteam3636.bunnybots2025.SparkFlex
import com.frcteam3636.bunnybots2025.TalonFX
import com.frcteam3636.bunnybots2025.subsystems.intake.IntakeIOReal.Constants.ACCELERATION
import com.frcteam3636.bunnybots2025.subsystems.intake.IntakeIOReal.Constants.CRUISE_VELOCITY
import com.frcteam3636.bunnybots2025.utils.math.*
import com.revrobotics.spark.SparkLowLevel
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

@Logged
open class ShooterInputs {
    var topVelocity = RotationsPerSecond.zero()
    var topCurrent = Amps.zero()
    var bottomVelocity = RotationsPerSecond.zero()
    var bottomCurrent = Amps.zero()
}

interface ShooterIO {
    fun setSpeed(upperPercent: Double, lowerPercent: Double)
    fun updateInputs(inputs: ShooterInputs)
}

class ShooterIOReal : ShooterIO {

    private val upperShooterMotor = SparkFlex(REVMotorControllerId.UpperShooterMotor, SparkLowLevel.MotorType.kBrushless)
    private val bottomShooterMotor = SparkFlex(REVMotorControllerId.LowerShooterMotor, SparkLowLevel.MotorType.kBrushless)

    override fun setSpeed(upperPercent: Double, lowerPercent: Double) {
        assert(upperPercent in -1.0..1.0)
        assert(lowerPercent in -1.0..1.0)
        upperShooterMotor.set(upperPercent)
        bottomShooterMotor.set(lowerPercent)
    }

    override fun updateInputs(inputs: ShooterInputs) {
        inputs.topVelocity = upperShooterMotor.encoder.velocity.rpm
        inputs.topCurrent = upperShooterMotor.outputCurrent.amps
        inputs.bottomVelocity = bottomShooterMotor.encoder.velocity.rpm
        inputs.bottomCurrent = bottomShooterMotor.outputCurrent.amps
    }
}

class ShooterIOSim: ShooterIO {
    override fun setSpeed(upperPercent: Double, lowerPercent: Double) {
        TODO("Not yet implemented")
    }

    override fun updateInputs(inputs: ShooterInputs) {
        TODO("Not yet implemented")
    }

}