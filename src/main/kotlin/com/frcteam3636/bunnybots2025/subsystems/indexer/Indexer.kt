package com.frcteam3636.bunnybots2025.subsystems.indexer

import com.ctre.phoenix6.BaseStatusSignal
import com.frcteam3636.bunnybots2025.Robot
import com.frcteam3636.bunnybots2025.subsystems.shooter.Shooter
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

    fun idle(): Command =
        runEnd(
            {
                if (Shooter.Flywheels.isDetected.asBoolean) {
                    io.setIndexerSpeed(-0.04)
                } else {
                    io.setIndexerSpeed(0.0)
                }
            },
            {
                io.setIndexerSpeed(0.0)
            }
        )

    fun index(): Command =
        startEnd(
            {
                io.setIndexerSpeed(0.8)
            },
            {
                io.setIndexerSpeed(0.0)
            }
        ).alongWith(
            Commands.run({
                if (!wasDetected && inputs.isDetected) {
                    wasDetected = true
                } else if (!inputs.isDetected) {
                    wasDetected = false
                }
            })
        ).finallyDo { ->
            wasDetected = false
        }

    fun slowIndex(): Command =
        startEnd(
            {
                io.setIndexerSpeed(0.2)
            },
            {
                io.setIndexerSpeed(0.0)
            }
        ).alongWith(
            Commands.run({
                if (!wasDetected && inputs.isDetected) {
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