package com.frcteam3636.bunnybots2025

import choreo.auto.AutoRoutine
import com.frcteam3636.bunnybots2025.subsystems.intake.Intake
import com.frcteam3636.bunnybots2025.subsystems.shooter.Shooter
import com.frcteam3636.bunnybots2025.subsystems.shooter.Target
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

object Autos {
    private fun intakeThenBulldoze(): Command {
        return Commands.sequence(
            Robot.doIntakeSequence().until {
                RobotState.heldPieces == 4
            },
            Intake.bulldoze().alongWith(Robot.blinkLimelight())
        )
    }

    fun scorePreloadLeft(): AutoRoutine {
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
            Robot.doShootSequence().withTimeout(SHOOT_TIMEOUT).until {
                RobotState.heldPieces == 0
            }
        )

        return routine
    }

    fun scorePreloadAndOnePatchLeft(): AutoRoutine {
        val routine = Robot.autoFactory.newRoutine("preloadAndOnePatch")

        val driveToZoo = routine.trajectory("LeftOne")
        val driveToPatchFromFirstScore = routine.trajectory("LeftPatchOne")
        val driveToZooFromFirstPatch = routine.trajectory("LeftScoreOne")
        val driveToPatchFromSecondScore = routine.trajectory("LeftPatchTwo")

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
                Robot.doShootSequence().withTimeout(SHOOT_TIMEOUT).until {
                    RobotState.heldPieces == 0
                },
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
            Commands.sequence(
                Robot.doShootSequence().withTimeout(SHOOT_TIMEOUT).until {
                    RobotState.heldPieces == 0
                },
                driveToPatchFromSecondScore.cmd()
            )
        )

        driveToPatchFromSecondScore.active().onTrue(
            intakeThenBulldoze()
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
                Robot.doShootSequence().withTimeout(SHOOT_TIMEOUT).until {
                    RobotState.heldPieces == 0
                },
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
                Robot.doShootSequence().withTimeout(SHOOT_TIMEOUT).until {
                    RobotState.heldPieces == 0
                },
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
                Robot.doShootSequence().withTimeout(SHOOT_TIMEOUT).until {
                    RobotState.heldPieces == 0
                },
                driveToPatchFromSecondScore.cmd()
            )
        )

        return routine
    }

    const val SHOOT_TIMEOUT = 3.0
    const val TIME_REMAINING_REQUIREMENT = 5
}