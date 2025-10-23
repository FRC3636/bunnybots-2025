package com.frcteam3636.bunnybots2025

import com.frcteam3636.bunnybots2025.subsystems.drivetrain.Drivetrain
import edu.wpi.first.wpilibj.smartdashboard.Field2d

object Dashboard {
    val field = Field2d()
//    val autoChooser = AutoBuilder.buildAutoChooser()!!

    fun update() {
        field.robotPose = Drivetrain.estimatedPose
    }
}
