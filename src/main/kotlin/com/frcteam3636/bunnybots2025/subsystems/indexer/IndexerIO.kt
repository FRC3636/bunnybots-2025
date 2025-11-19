package com.frcteam3636.bunnybots2025.subsystems.indexer

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.CANrangeConfiguration
import com.ctre.phoenix6.signals.UpdateModeValue
import com.frcteam3636.bunnybots2025.CANrange
import com.frcteam3636.bunnybots2025.CTREDeviceId
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
open class IndexerInputs {
    var indexerVelocity = RotationsPerSecond.zero()!!
    var indexerCurrent = Amps.zero()!!
    var indexerTemperature = Celsius.zero()!!
    var isDetected = false
}

interface IndexerIO {
    fun setIndexerSpeed(percentage: Double)
    fun updateInputs(inputs: IndexerInputs)

    val signals: Array<BaseStatusSignal>
        get() = emptyArray()
}

class IndexerIOReal : IndexerIO {
    private var indexerMotor = SparkFlex(REVDeviceId.IndexerMotor, SparkLowLevel.MotorType.kBrushless)
    private var canRange = CANrange(CTREDeviceId.CANRangeIndexer).apply {
        configurator.apply(
            CANrangeConfiguration().apply {
                ProximityParams.ProximityThreshold = 0.35 // fix
                ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz
            }
        )
    }

    private val detectedSignal = canRange.isDetected

    init {
        BaseStatusSignal.setUpdateFrequencyForAll(100.0, detectedSignal)
        canRange.optimizeBusUtilization()
    }

    override fun updateInputs(inputs: IndexerInputs) {
        inputs.indexerCurrent = indexerMotor.outputCurrent.amps
        inputs.indexerVelocity = indexerMotor.encoder.velocity.rpm
        inputs.indexerTemperature = indexerMotor.motorTemperature.celsius
    }

    override val signals: Array<BaseStatusSignal>
        get() = arrayOf(detectedSignal)

    override fun setIndexerSpeed(percentage: Double) {
        assert(percentage in -1.0..1.0)
        indexerMotor.set(percentage)
    }
}

class IndexerIOSim : IndexerIO {
    private val indexerMotorSystem = LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.0001, 1.0)
    private val indexerMotor = DCMotorSim(indexerMotorSystem, DCMotor.getNeoVortex(1))

    override fun setIndexerSpeed(percentage: Double) {
        indexerMotor.inputVoltage = 12.0 * percentage
    }

    override fun updateInputs(inputs: IndexerInputs) {
        inputs.indexerVelocity = indexerMotor.angularVelocity
        inputs.indexerCurrent = indexerMotor.currentDrawAmps.amps
    }
}