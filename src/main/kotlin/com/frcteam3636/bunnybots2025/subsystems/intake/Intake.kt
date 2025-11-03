package com.frcteam3636.bunnybots2025.subsystems.intake

import com.ctre.phoenix6.BaseStatusSignal
import com.frcteam3636.bunnybots2025.Robot
import com.frcteam3636.bunnybots2025.utils.math.degrees
import com.frcteam3636.bunnybots2025.utils.math.inDegrees
import com.frcteam3636.bunnybots2025.utils.math.rotations
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

object Intake : Subsystem {
    private var io: IntakeIO = when (Robot.model) {
        Robot.Model.SIMULATION -> IntakeIOSim()
        Robot.Model.COMPETITION -> IntakeIOReal()
    }

    var inputs = LoggedIntakeInputs()

    var mechanism = LoggedMechanism2d(100.0, 200.0)
    var intakeAngleLigament = LoggedMechanismLigament2d("Intake Ligament", 50.0, 0.0, 5.0, Color8Bit(Color.kGreen))

    init {
        mechanism.getRoot("Intake", 50.0, 150.0).apply {
            append(intakeAngleLigament)
        }
    }

    private val pivotDisabledAlert = Alert("The intake pivot has been disabled due to an error. To re-enable please restart robot code",
        Alert.AlertType.kError)

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Intake", inputs)
        intakeAngleLigament.angle = inputs.pivotPosition.inDegrees() + 90.0
        Logger.recordOutput("Intake/Pivot/Mechanism", mechanism)

        // FIXME: tune these
        if ((inputs.pivotPosition > 100.degrees || inputs.pivotPosition < (-1).degrees) && !inputs.pivotDisabled) {
            io.disablePivot()
            pivotDisabledAlert.set(true)
        }
    }

    fun intake(): Command =
        startEnd(
            {
                io.setRollerSpeed(0.7)
                io.setPivotPosition(Position.Deployed.angle)
            },
            {
                io.setRollerSpeed(0.0)
                io.setPivotPosition(Position.Stowed.angle)
            }
        )

    fun outtake(): Command =
        startEnd(
            {
                io.setRollerSpeed(-0.5)
                println("Deploying")
                io.setPivotPosition(Position.Deployed.angle)
            },
            {
                io.setRollerSpeed(0.0)
                io.setPivotPosition(Position.Stowed.angle)
            }
        )

    enum class Position(val angle: Angle) {
        Stowed(0.rotations),
        Deployed(5.rotations); // FIXME: Placeholder
    }

    fun getStatusSignals(): MutableList<BaseStatusSignal> {
        return io.getSignals()
    }
}