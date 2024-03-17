package org.firstinspires.ftc.teamcode.extensions

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.Point

object PoseExtensions {
    fun Pose2d.toPoint(): Point {
        return Point(this.x, this.y)
    }
}