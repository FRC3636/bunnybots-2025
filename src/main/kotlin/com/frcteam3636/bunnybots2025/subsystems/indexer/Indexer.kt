package com.frcteam3636.bunnybots2025.subsystems.indexer

import com.ctre.phoenix6.BaseStatusSignal
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

    var currentTarget = Target.STOP

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Indexer", inputs)

        io.setIndexerSpeed(currentTarget.profile.getPercent())
    }

    val signals: Array<BaseStatusSignal>
        get() = io.signals

    fun setTarget(target: Target): Command =
        runOnce { currentTarget = target }

    enum class Target(val profile: IndexerProfile) {
        INDEX(
            IndexerProfile { 0.4 }
        ),
        SLOWINDEX(
            IndexerProfile { 0.1 }
        ),
        OUTDEX(
            IndexerProfile { -0.4 }
        ),
        STOP(
            IndexerProfile { 0.0 }
        )
    }

    data class IndexerProfile(
        val getPercent: () -> Double
    )
}