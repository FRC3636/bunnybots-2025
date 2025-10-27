package com.frcteam3636.bunnybots2025.subsystems.shooter

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.bunnybots2025.CTREDeviceId
import com.frcteam3636.bunnybots2025.TalonFX
import com.frcteam3636.bunnybots2025.subsystems.intake.IntakeIOReal.Constants.ACCELERATION
import com.frcteam3636.bunnybots2025.subsystems.intake.IntakeIOReal.Constants.CRUISE_VELOCITY
import com.frcteam3636.bunnybots2025.utils.math.*
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import org.littletonrobotics.junction.Logger
import org.team9432.annotation.Logged

@Logged
open class PivotInputs {
    var pivotAngle = Rotations.zero()
    var pivotCurrent = Amps.zero()
    var pivotVelocity = RotationsPerSecond.zero()
}

interface PivotIO {
    fun turnToAngle(angle: Angle)
    fun setBrakeMode(enabled: Boolean) {}
    fun getStatusSignals(): MutableList<BaseStatusSignal> {
        return mutableListOf()
    }
    fun updateInputs(inputs: PivotInputs)
}

class PivotIOReal: PivotIO {
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
        })
    }

    private val positionSignal = shooterPivotMotor.position
    private val currentSignal = shooterPivotMotor.supplyCurrent
    private val velocitySignal = shooterPivotMotor.velocity

    override fun turnToAngle(angle: Angle) {
        val positionControl = MotionMagicTorqueCurrentFOC(0.0).apply {
            Slot = 0
            Position = angle.inRotations()
        }
        shooterPivotMotor.setControl(positionControl)
    }

    override fun setBrakeMode(enabled: Boolean) {
        shooterPivotMotor.setNeutralMode(
            if (enabled) {
                NeutralModeValue.Brake
            } else {
                NeutralModeValue.Coast
            }
        )
    }

    override fun getStatusSignals(): MutableList<BaseStatusSignal> {
        return mutableListOf(positionSignal, currentSignal)
    }

    override fun updateInputs(inputs: PivotInputs) {
        inputs.pivotAngle = positionSignal.value // multiply by driver/driven gear ratio?
        inputs.pivotCurrent = currentSignal.value
        inputs.pivotVelocity = velocitySignal.value
    }

    internal companion object Constants {
        private val PID_GAINS = PIDGains(6.0, 0.0, 0.0)
        private const val PROFILE_ACCELERATION = 50.0
        private const val PROFILE_JERK = 0.0
        private const val PROFILE_VELOCITY = 25.0
    }
}

class PivotIOSim: PivotIO {
    override fun turnToAngle(angle: Angle) {
        TODO("Not yet implemented")
    }

    override fun updateInputs(inputs: PivotInputs) {
        TODO("Not yet implemented")
    }

}

