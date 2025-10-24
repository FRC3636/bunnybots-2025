package com.frcteam3636.bunnybots2025.subsystems.intake

import com.ctre.phoenix6.BaseStatusSignal
import com.frcteam3636.bunnybots2025.Robot
import com.frcteam3636.bunnybots2025.utils.math.inDegrees
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object Intake: Subsystem {
    private var io: IntakeIO = when (Robot.model) {
        Robot.Model.SIMULATION -> IntakeIOSim()
        Robot.Model.COMPETITION -> IntakeIOReal()
    }

    var inputs = LoggedIntakeInputs()

    var mechanism = Mechanism2d(100.0, 200.0)
    var intakeAngleLigament = MechanismLigament2d("Intake Ligament", 50.0, 90.0, 5.0, Color8Bit(Color.kGreen))

    init {
        mechanism.getRoot("Intake", 50.0, 150.0).apply {
            append(intakeAngleLigament)
        }
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Intake", inputs)
        intakeAngleLigament.angle = inputs.pivotPosition.inDegrees()
        Logger.recordOutput("Pivot Angle", intakeAngleLigament.angle)
    }

    fun intake(): Command =
        startEnd(
            {
                io.setSpeed(0.7)
            },
            {
                io.setSpeed(0.0)
            }
        )

    fun outtake(): Command =
        startEnd(
            {
                io.setSpeed(-0.5)
            },
            {
                io.setSpeed(0.0)
            }
        )

    fun getStatusSignals(): MutableList<BaseStatusSignal> {
        return io.getSignals()
    }
}