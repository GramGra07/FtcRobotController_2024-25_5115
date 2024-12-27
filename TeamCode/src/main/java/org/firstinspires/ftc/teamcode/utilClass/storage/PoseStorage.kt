package org.firstinspires.ftc.teamcode.utilClass.storage

import com.acmerobotics.roadrunner.Pose2d
import org.firstinspires.ftc.teamcode.followers.pedroPathing.localization.Pose
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.Point

class PoseStorage {
    companion object {
        var currentPose: Pose2d = Pose2d(0.0, 0.0, 0.0)

        fun Pose.toPoint(): Point {
            return Point(this.x, this.y)
        }
    }
}