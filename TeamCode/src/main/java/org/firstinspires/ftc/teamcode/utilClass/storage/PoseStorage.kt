package org.firstinspires.ftc.teamcode.utilClass.storage

import org.firstinspires.ftc.teamcode.followers.pedroPathing.localization.Pose
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.Point

class PoseStorage {
    companion object {
        var currentPose: Pose = Pose(0.0, 0.0, 0.0)
        var currentPoint: Point = currentPose.toPoint()
        var currentHeading: Double = currentPose.heading

        fun Pose.toPoint(): Point {
            return Point(this.x, this.y)
        }
    }
}