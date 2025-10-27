package com.frcteam3636.bunnybots2025.subsystems.indexer

import com.frcteam3636.bunnybots2025.Robot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object Indexer : Subsystem {
    private var io: IndexerIO = when (Robot.model) {
        Robot.Model.SIMULATION -> IndexerIOSim()
        Robot.Model.COMPETITION -> IndexerIOReal()
    }

    var inputs = LoggedIndexerInputs()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Indexer", inputs)
    }

    fun intake(): Command =
        startEnd(
            {
                io.setIndexerSpeed(0.7)
            },
            {
                io.setIndexerSpeed(0.0)
            }
        )

    fun outtake(): Command =
        startEnd(
            {
                io.setIndexerSpeed(-0.5)
            },
            {
                io.setIndexerSpeed(0.0)
            }
        )
}