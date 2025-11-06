package com.frcteam3636.bunnybots2025

import choreo.auto.AutoRoutine
import com.frcteam3636.bunnybots2025.subsystems.intake.Intake
import com.frcteam3636.bunnybots2025.subsystems.shooter.Shooter
import com.frcteam3636.bunnybots2025.subsystems.shooter.Target
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

object Autos {
    private fun intakeThenBulldoze(): Command {
        return Commands.sequence(
            Robot.doIntakeSequence().until {
                RobotState.heldPieces == 4
            },
            Intake.bulldoze()
        )
    }

    fun scorePreload(): AutoRoutine {
        val routine = Robot.autoFactory.newRoutine("preload")

        val driveToZoo = routine.trajectory("LeftOne")

        if (Robot.model == Robot.Model.SIMULATION) {
            routine.active().onTrue(driveToZoo.resetOdometry())
        }

        routine.active().onTrue(
            driveToZoo.cmd()
        )

        driveToZoo.active().onTrue(
            Shooter.Pivot.setTarget(Target.AIM)
        )

        driveToZoo.done().onTrue(
            Robot.doShootSequence().withTimeout(SHOOT_TIMEOUT)
        )

        return routine
    }

    fun scorePreloadAndOnePatch(): AutoRoutine {
        val routine = Robot.autoFactory.newRoutine("preloadAndOnePatch")

        val driveToZoo = routine.trajectory("LeftOne")
        val driveToPatchFromFirstScore = routine.trajectory("LeftPatchOne")
        val driveToZooFromFirstPatch = routine.trajectory("LeftScoreOne")

        if (Robot.model == Robot.Model.SIMULATION) {
            routine.active().onTrue(driveToZoo.resetOdometry())
        }

        routine.active().onTrue(
            driveToZoo.cmd()
        )

        driveToZoo.active().onTrue(
            Shooter.Pivot.setTarget(Target.AIM)
        )

        driveToZoo.done().onTrue(
            Commands.sequence(
                Robot.doShootSequence().withTimeout(SHOOT_TIMEOUT),
                driveToPatchFromFirstScore.cmd()
            )
        )

        driveToPatchFromFirstScore.active().whileTrue(
            intakeThenBulldoze()
        )

        driveToPatchFromFirstScore.done().onTrue(
            driveToZooFromFirstPatch.cmd()
        )

        driveToZooFromFirstPatch.active().onTrue(
            Robot.doShootSequence().withTimeout(SHOOT_TIMEOUT)
        )

        return routine
    }

    const val SHOOT_TIMEOUT = 3.0
}