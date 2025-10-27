package com.frcteam3636.bunnybots2025.subsystems.indexer

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
}

interface IndexerIO {
    fun setIndexerSpeed(percentage: Double)
    fun updateInputs(inputs: IndexerInputs)
}

class IndexerIOReal : IndexerIO {
    private var indexerMotor = SparkFlex(REVMotorControllerId.IndexerMotor, SparkLowLevel.MotorType.kBrushless)

    override fun updateInputs(inputs: IndexerInputs) {
        inputs.indexerCurrent = indexerMotor.outputCurrent.amps
        inputs.indexerVelocity = indexerMotor.encoder.velocity.rpm
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