package com.frcteam3636.bunnybots2025.subsystems.indexer

import com.ctre.phoenix6.BaseStatusSignal
import com.frcteam3636.bunnybots2025.Robot
import com.frcteam3636.bunnybots2025.RobotState
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object Indexer : Subsystem {
    private var io: IndexerIO = when (Robot.model) {
        Robot.Model.SIMULATION -> IndexerIOSim()
        Robot.Model.COMPETITION -> IndexerIOReal()
    }

    var inputs = LoggedIndexerInputs()

    private var wasDetected = false

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Indexer", inputs)
    }

    fun index(): Command =
        startEnd(
            {
                io.setIndexerSpeed(0.7)
            },
            {
                io.setIndexerSpeed(0.0)
            }
        ).alongWith(
            Commands.run({
                if (!wasDetected && inputs.isDetected) {
                    RobotState.heldPieces++
                    wasDetected = true
                } else if (!inputs.isDetected) {
                    wasDetected = false
                }
            })
        ).finallyDo { ->
            wasDetected = false
        }

    fun outtake(): Command =
        startEnd(
            {
                io.setIndexerSpeed(-0.5)
            },
            {
                io.setIndexerSpeed(0.0)
            }
        ).alongWith(
            Commands.run({
                if (!wasDetected && inputs.isDetected) {
                    RobotState.heldPieces--
                    wasDetected = true
                } else if (!inputs.isDetected) {
                    wasDetected = false
                }
            })
        ).finallyDo { ->
            wasDetected = false
        }

    val signals: Array<BaseStatusSignal>
        get() = io.signals
}