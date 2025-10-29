package com.frcteam3636.bunnybots2025.subsystems.indexer

import com.ctre.phoenix6.BaseStatusSignal
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
import org.team9432.annotation.Logged

@Logged
open class IndexerInputs {
    var indexerVelocity = RotationsPerSecond.zero()!!
    var indexerCurrent = Amps.zero()!!
    var isDetected = false
}

interface IndexerIO {
    fun setIndexerSpeed(percentage: Double)
    fun updateInputs(inputs: IndexerInputs)
    fun getSignals(): MutableList<BaseStatusSignal> {
        return mutableListOf()
    }
}

class IndexerIOReal : IndexerIO {
    private var indexerMotor = SparkFlex(REVMotorControllerId.IndexerMotor, SparkLowLevel.MotorType.kBrushless)
    private var canRange = CANrange(CTREDeviceId.CANRangeIndexer.num).apply {
        configurator.apply(
            CANrangeConfiguration().apply {
                ProximityParams.ProximityThreshold = 0.35 // fix
                FovParams.FOVCenterY = 10.0 // fix
                FovParams.FOVRangeY = 7.0 // fix
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
    }

    override fun getSignals(): MutableList<BaseStatusSignal> {
        return mutableListOf(detectedSignal)
    }

    override fun setIndexerSpeed(percentage: Double) {
        assert(percentage in -1.0..1.0)
        indexerMotor.set(percentage)
    }
}

class IndexerIOSim : IndexerIO {
    override fun setIndexerSpeed(percentage: Double) {
        TODO("Not yet implemented")
    }

    override fun updateInputs(inputs: IndexerInputs) {
        TODO("Not yet implemented")
    }
}