package com.frcteam3636.bunnybots2025.utils

import com.frcteam3636.bunnybots2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.bunnybots2025.utils.math.degrees
import com.frcteam3636.bunnybots2025.utils.math.inRadians
import com.frcteam3636.bunnybots2025.utils.math.inches
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units
import org.littletonrobotics.junction.Logger
import kotlin.math.abs
import kotlin.math.sin

fun getObjectPose(): Pose2d {
    val robotPose = Pose3d(Drivetrain.estimatedPose)
    val limelightPose = robotPose.transformBy(LIMELIGHT_TRANSFORM)
    Logger.recordOutput("Drivetrain/Object Camera Pose", limelightPose)

    var ty = 0.0
    var tx = 0.0
    var estimatedDistance = 0.0

    if (LimelightHelpers.getTV(OBJECT_DETECTOR_NAME)) {
        ty = LimelightHelpers.getTY(OBJECT_DETECTOR_NAME)
        tx = LimelightHelpers.getTX(OBJECT_DETECTOR_NAME)
        estimatedDistance = abs(
            (limelightPose.z + CAMERA_Z_OFFSET)
                    / sin(Units.degreesToRadians(ty) - limelightPose.rotation.y)
        )
    } else {
        Logger.recordOutput("Drivetrain/Carrot Pose", Pose2d.kZero)
        return Pose2d.kZero
    }

    val limelightRelative = Translation3d(
        estimatedDistance,
        Rotation3d(
            0.0,
            Units.degreesToRadians(ty.unaryMinus()),
            Units.degreesToRadians(tx.unaryMinus()),
        )
    )

    val fieldRelative = limelightRelative.rotateBy(limelightPose.rotation).plus(limelightPose.translation)

    val pose = Pose3d(fieldRelative, Rotation3d.kZero).toPose2d()
    Logger.recordOutput("Drivetrain/Carrot Pose", pose)
    return pose
}

const val OBJECT_DETECTOR_NAME = "limelight"
const val CAMERA_Z_OFFSET = 0.125
val LIMELIGHT_TRANSFORM = Transform3d(
    16.inches, 0.0.inches, 5.inches, Rotation3d(
        0.0, -10.0.degrees.inRadians(), 0.0
    )
)