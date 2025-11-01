package com.frcteam3636.bunnybots2025.subsystems.shooter

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.CANrangeConfiguration
import com.ctre.phoenix6.signals.UpdateModeValue
import com.frcteam3636.bunnybots2025.CANrange
import com.frcteam3636.bunnybots2025.CTREDeviceId
import com.frcteam3636.bunnybots2025.REVMotorControllerId
import com.frcteam3636.bunnybots2025.SparkFlex
import com.frcteam3636.bunnybots2025.utils.math.MotorFFGains
import com.frcteam3636.bunnybots2025.utils.math.PIDGains
import com.frcteam3636.bunnybots2025.utils.math.SimpleMotorFeedforward
import com.frcteam3636.bunnybots2025.utils.math.amps
import com.frcteam3636.bunnybots2025.utils.math.celsius
import com.frcteam3636.bunnybots2025.utils.math.inRPM
import com.frcteam3636.bunnybots2025.utils.math.inVolts
import com.frcteam3636.bunnybots2025.utils.math.rpm
import com.revrobotics.spark.ClosedLoopSlot
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkFlexConfig
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

@Logged
open class FlywheelInputs {
    var topVelocity = RotationsPerSecond.zero()!!
    var topCurrent = Amps.zero()!!
    var bottomVelocity = RotationsPerSecond.zero()!!
    var bottomCurrent = Amps.zero()!!
    var topTemperature = Celsius.zero()!!
    var bottomTemperature = Celsius.zero()!!
    var isDetected = false
}

interface FlywheelIO {
    fun setSpeed(upperPercent: Double, lowerPercent: Double)
    fun setVoltage(upperVoltage: Voltage, lowerVoltage: Voltage)
    fun setVoltage(voltage: Voltage)
    fun setVelocity(velocity: AngularVelocity)
    fun updateInputs(inputs: FlywheelInputs)
    fun getStatusSignals(): MutableList<BaseStatusSignal> {
        return mutableListOf()
    }
}

class FlywheelIOReal : FlywheelIO {

    private val upperShooterMotor =
        SparkFlex(REVMotorControllerId.UpperShooterMotor, SparkLowLevel.MotorType.kBrushless).apply {
            configure(SparkFlexConfig().apply {
                idleMode(SparkBaseConfig.IdleMode.kCoast)

                closedLoop.apply {
                    pid(LOWER_PID_GAINS.p, LOWER_PID_GAINS.i, LOWER_PID_GAINS.d)
                }
            }, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        }
    private val lowerShooterMotor =
        SparkFlex(REVMotorControllerId.LowerShooterMotor, SparkLowLevel.MotorType.kBrushless).apply {
            configure(SparkFlexConfig().apply {
                idleMode(SparkBaseConfig.IdleMode.kCoast)

                closedLoop.apply {
                    pid(UPPER_PID_GAINS.p, UPPER_PID_GAINS.i, UPPER_PID_GAINS.d)
                }
            }, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        }

    private var upperFFController = SimpleMotorFeedforward(UPPER_FF_GAINS)
    private var lowerFFController = SimpleMotorFeedforward(LOWER_FF_GAINS)


    private var canRange = CANrange(CTREDeviceId.CANRangeShooter).apply {
        configurator.apply(
            CANrangeConfiguration().apply {
                ProximityParams.ProximityThreshold = 0.1 // fix
                ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz
            }
        )
    }

    private val detectedSignal = canRange.isDetected

    init {
        BaseStatusSignal.setUpdateFrequencyForAll(100.0, detectedSignal)
        canRange.optimizeBusUtilization()
    }

    override fun setSpeed(upperPercent: Double, lowerPercent: Double) {
        assert(upperPercent in -1.0..1.0)
        assert(lowerPercent in -1.0..1.0)
        upperShooterMotor.set(upperPercent)
        lowerShooterMotor.set(lowerPercent)
    }

    override fun setVoltage(upperVoltage: Voltage, lowerVoltage: Voltage) {
        assert(upperVoltage.inVolts() in -13.0..13.0)
        assert(lowerVoltage.inVolts() in -13.0..13.0)
        upperShooterMotor.setVoltage(upperVoltage)
        lowerShooterMotor.setVoltage(lowerVoltage)
    }

    override fun setVoltage(voltage: Voltage) {
        assert(voltage.inVolts() in -13.0..13.0)
        upperShooterMotor.setVoltage(voltage)
        lowerShooterMotor.setVoltage(voltage)
    }

    override fun setVelocity(velocity: AngularVelocity) {
        upperShooterMotor.closedLoopController.setReference(velocity.inRPM(), SparkBase.ControlType.kVelocity,
            ClosedLoopSlot.kSlot0, upperFFController.calculate(velocity.inRPM()))
        lowerShooterMotor.closedLoopController.setReference(velocity.inRPM(), SparkBase.ControlType.kVelocity,
            ClosedLoopSlot.kSlot0, lowerFFController.calculate(velocity.inRPM()))
    }

    override fun updateInputs(inputs: FlywheelInputs) {
        inputs.topVelocity = upperShooterMotor.encoder.velocity.rpm
        inputs.topCurrent = upperShooterMotor.outputCurrent.amps
        inputs.bottomVelocity = lowerShooterMotor.encoder.velocity.rpm
        inputs.bottomCurrent = lowerShooterMotor.outputCurrent.amps
        inputs.isDetected = detectedSignal.value
        inputs.topTemperature = upperShooterMotor.motorTemperature.celsius
        inputs.bottomTemperature = lowerShooterMotor.motorTemperature.celsius
    }

    override fun getStatusSignals(): MutableList<BaseStatusSignal> {
        return mutableListOf(detectedSignal)
    }

    companion object Constants {
        val UPPER_PID_GAINS = PIDGains()
        val LOWER_PID_GAINS = PIDGains()
        val UPPER_FF_GAINS = MotorFFGains()
        val LOWER_FF_GAINS = MotorFFGains()
    }
}

class FlywheelIOSim : FlywheelIO {
    override fun setSpeed(upperPercent: Double, lowerPercent: Double) {
        TODO("Not yet implemented")
    }

    override fun updateInputs(inputs: FlywheelInputs) {
        TODO("Not yet implemented")
    }

    override fun setVoltage(upperVoltage: Voltage, lowerVoltage: Voltage) {
        TODO("Not yet implemented")
    }

    override fun setVoltage(voltage: Voltage) {
        TODO("Not yet implemented")
    }

    override fun setVelocity(velocity: AngularVelocity) {
        TODO("Not yet implemented")
    }
}