package com.frcteam3636.bunnybots2025.subsystems.shooter

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.CANrangeConfiguration
import com.ctre.phoenix6.signals.UpdateModeValue
import com.frcteam3636.bunnybots2025.CANrange
import com.frcteam3636.bunnybots2025.CTREDeviceId
import com.frcteam3636.bunnybots2025.REVDeviceId
import com.frcteam3636.bunnybots2025.SparkFlex
import com.frcteam3636.bunnybots2025.utils.math.*
import com.revrobotics.spark.ClosedLoopSlot
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkFlexConfig
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.simulation.DCMotorSim
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
    fun setVoltage(voltage: Voltage)
    fun setVelocity(velocity: AngularVelocity)
    fun updateInputs(inputs: FlywheelInputs)

    val signals: Array<BaseStatusSignal>
        get() = emptyArray()
}

class FlywheelIOReal : FlywheelIO {

    private val upperShooterMotor =
        SparkFlex(REVDeviceId.UpperShooterMotor, SparkLowLevel.MotorType.kBrushless).apply {
            configure(SparkFlexConfig().apply {
                idleMode(SparkBaseConfig.IdleMode.kCoast)

                closedLoop.apply {
                    UPPER_PID_GAINS.toRevLib()
                }
            }, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        }
    private val lowerShooterMotor =
        SparkFlex(REVDeviceId.LowerShooterMotor, SparkLowLevel.MotorType.kBrushless).apply {
            configure(SparkFlexConfig().apply {
                idleMode(SparkBaseConfig.IdleMode.kCoast)

                closedLoop.apply {
                    LOWER_PID_GAINS.toRevLib()
                }

            }, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        }

    private var upperFFController = SimpleMotorFeedforward(UPPER_FF_GAINS)
    private var lowerFFController = SimpleMotorFeedforward(LOWER_FF_GAINS)


    // TODO: Move this into the feeder subsystem. Doesn't really matter but it makes more sense from an organization level.
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

    override fun setVoltage(voltage: Voltage) {
        assert(voltage.inVolts() in -12.0..12.0)
        upperShooterMotor.setVoltage(voltage)
        lowerShooterMotor.setVoltage(voltage)
    }

    override fun setVelocity(velocity: AngularVelocity) {
        val upperFFOutput = upperFFController.calculate(velocity.inRPM())
        val lowerFFOutput = lowerFFController.calculate(velocity.inRPM())
        upperShooterMotor.closedLoopController.setReference(
            velocity.inRPM(), SparkBase.ControlType.kVelocity,
            ClosedLoopSlot.kSlot0, upperFFOutput
        )
        lowerShooterMotor.closedLoopController.setReference(
            velocity.inRPM(), SparkBase.ControlType.kVelocity,
            ClosedLoopSlot.kSlot0, lowerFFOutput
        )
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

    override val signals: Array<BaseStatusSignal>
        get() = arrayOf(detectedSignal)

    companion object Constants {
        val UPPER_PID_GAINS = PIDGains()
        val UPPER_FF_GAINS = MotorFFGains()
        val LOWER_PID_GAINS = PIDGains()
        val LOWER_FF_GAINS = MotorFFGains()
    }
}

class FlywheelIOSim : FlywheelIO {

    private val flywheelSim = LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.0001, 1.0)
    private val upperFlywheelMotor = DCMotorSim(flywheelSim, DCMotor.getNeoVortex(1))
    private val lowerFlywheelMotor = DCMotorSim(flywheelSim, DCMotor.getNeoVortex(1))

    override fun setSpeed(upperPercent: Double, lowerPercent: Double) {
        upperFlywheelMotor.inputVoltage = upperPercent * 12.0
        lowerFlywheelMotor.inputVoltage = lowerPercent * 12.0
    }

    override fun updateInputs(inputs: FlywheelInputs) {
        inputs.topVelocity = upperFlywheelMotor.angularVelocity
        inputs.bottomVelocity = lowerFlywheelMotor.angularVelocity
        inputs.topCurrent = upperFlywheelMotor.currentDrawAmps.amps
        inputs.bottomCurrent = lowerFlywheelMotor.currentDrawAmps.amps
    }

    override fun setVoltage(voltage: Voltage) {
        upperFlywheelMotor.inputVoltage = voltage.inVolts()
        lowerFlywheelMotor.inputVoltage = voltage.inVolts()
    }

    override fun setVelocity(velocity: AngularVelocity) {
        upperFlywheelMotor.setAngularVelocity(velocity.inRadiansPerSecond())
        lowerFlywheelMotor.setAngularVelocity(velocity.inRadiansPerSecond())
    }
}