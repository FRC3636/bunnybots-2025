@file:Suppress("DuplicatedCode")

package com.frcteam3636.bunnybots2025

import choreo.auto.AutoRoutine
import com.frcteam3636.bunnybots2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.bunnybots2025.subsystems.indexer.Indexer
import com.frcteam3636.bunnybots2025.subsystems.intake.Intake
import com.frcteam3636.bunnybots2025.subsystems.shooter.Shooter
import com.frcteam3636.bunnybots2025.subsystems.shooter.Target
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

object Autos {
    private fun intakeThenBulldoze(): Command {
        return Commands.sequence(
            Robot.doIntakeSequence().withTimeout(BULLDOZE_TIMEOUT),
            Intake.bulldoze()
        )
    }

    private fun doShootSequence(): Command {
        return Commands.parallel(
            Shooter.Flywheels.shoot(),
            Commands.sequence(
                Commands.waitUntil(Shooter.Flywheels.atDesiredVelocity),
                Commands.waitUntil(Shooter.Pivot.atDesiredPosition),
                Commands.parallel(
                    Shooter.Feeder.feed(),
                    Indexer.index(),
                    Intake.intake()
                )
            )
        )
    }

    fun scorePreloadLeft(): AutoRoutine {
        val routine = Robot.autoFactory.newRoutine("preload")

        val driveToZoo = routine.trajectory("LeftOne")

//        routine.active().onTrue(driveToZoo.resetOdometry())

        routine.active().onTrue(
            driveToZoo.cmd()
        )

        driveToZoo.active().onTrue(
            Shooter.Pivot.setTarget(Target.AIM)
        )

        driveToZoo.done().onTrue(
            Commands.sequence(
//                Drivetrain.alignToZoo(),
                doShootSequence().withTimeout(SHOOT_TIMEOUT)
            )
        )

        return routine
    }

    fun scorePreloadAndOnePatchLeft(): AutoRoutine {
        val routine = Robot.autoFactory.newRoutine("preloadAndOnePatch")

        val driveToZoo = routine.trajectory("LeftOne")
        val driveToPatchFromFirstScore = routine.trajectory("LeftPatchOne")
        val driveToZooFromFirstPatch = routine.trajectory("LeftScoreOne")
        val driveToPatchFromSecondScore = routine.trajectory("LeftPatchTwo")

//        routine.active().onTrue(driveToZoo.resetOdometry())

        routine.active().onTrue(
            driveToZoo.cmd()
        )

        driveToZoo.active().onTrue(
            Shooter.Pivot.setTarget(Target.AIM)
        )

        driveToZoo.done().onTrue(
            doShootSequence().withTimeout(SHOOT_TIMEOUT)
                .andThen(driveToPatchFromFirstScore.cmd())
            )

        driveToPatchFromFirstScore.active().whileTrue(
            intakeThenBulldoze()
        )

        driveToPatchFromFirstScore.done().onTrue(
            driveToZooFromFirstPatch.cmd()
        )

        driveToZooFromFirstPatch.active().onTrue(
            Commands.sequence(
//                Drivetrain.alignToZoo(),
                doShootSequence().withTimeout(SHOOT_TIMEOUT),
                driveToPatchFromSecondScore.cmd()
            )
        )

        driveToPatchFromSecondScore.active().onTrue(
            intakeThenBulldoze()
        )

        return routine
    }

    fun scorePreloadAndOnePatchLeftRewrote(): AutoRoutine {
        val routine = Robot.autoFactory.newRoutine("preloadAndOnePatch")

        val driveToZoo = routine.trajectory("LeftOne")
        val driveToPatchFromFirstScore = routine.trajectory("LeftPatchOne")
        val driveToZooFromFirstPatch = routine.trajectory("LeftScoreOne")
        val driveToPatchFromSecondScore = routine.trajectory("LeftPatchTwo")

        routine.active().onTrue(
            Commands.sequence(
                driveToZoo.cmd(),
                Shooter.Pivot.setTarget(Target.AIM),
                doShootSequence().withTimeout(SHOOT_TIMEOUT),
                Commands.race(
                    driveToPatchFromFirstScore.cmd(),
                    intakeThenBulldoze()
                ),
                driveToZooFromFirstPatch.cmd(),
                doShootSequence().withTimeout(SHOOT_TIMEOUT),
                Commands.race(
                    driveToPatchFromSecondScore.cmd(),
                    intakeThenBulldoze()
                )
            )
        )

        return routine
    }

    fun cantStopWontStopLeft(): AutoRoutine {
        val routine = Robot.autoFactory.newRoutine("cantStopWontStop")

        val driveToZoo = routine.trajectory("LeftOne")
        val driveToPatchFromFirstScore = routine.trajectory("LeftPatchOne")
        val driveToZooFromFirstPatch = routine.trajectory("LeftScoreOne")
        val driveToPatchFromSecondScore = routine.trajectory("LeftPatchTwo")
        val driveToZooFromSecondPatch = routine.trajectory("LeftScoreTwo")

        routine.active().onTrue(driveToZoo.resetOdometry())

        routine.active().onTrue(
            driveToZoo.cmd()
        )

        driveToZoo.active().onTrue(
            Shooter.Pivot.setTarget(Target.AIM)
        )

        driveToZoo.done().onTrue(
            Commands.sequence(
                Drivetrain.alignToZoo(),
                doShootSequence().withTimeout(SHOOT_TIMEOUT),
                driveToPatchFromFirstScore.cmd()
            )
        )

        driveToPatchFromFirstScore.active().whileTrue(
            intakeThenBulldoze()
        )

        driveToPatchFromFirstScore.done().onTrue(
            driveToZooFromFirstPatch.cmd()
        )

        driveToZooFromFirstPatch.done().onTrue(
            Commands.sequence(
                Drivetrain.alignToZoo(),
                Robot.doShootSequence().withTimeout(SHOOT_TIMEOUT),
                driveToPatchFromSecondScore.cmd()
            )
        )

        driveToPatchFromSecondScore.active().whileTrue(
            intakeThenBulldoze()
        )

        driveToPatchFromSecondScore.done().onTrue(
            driveToZooFromSecondPatch.cmd().onlyIf {
                DriverStation.getMatchTime() > TIME_REMAINING_REQUIREMENT
            }
        )

        driveToZooFromSecondPatch.done().onTrue(
            Commands.sequence(
                Drivetrain.alignToZoo(),
                Robot.doShootSequence().withTimeout(SHOOT_TIMEOUT),
                driveToPatchFromSecondScore.cmd()
            )
        )

        return routine
    }

    const val SHOOT_TIMEOUT = 5.0
    const val BULLDOZE_TIMEOUT = 2.0
    const val TIME_REMAINING_REQUIREMENT = 5
}