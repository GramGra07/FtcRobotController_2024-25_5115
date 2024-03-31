package org.firstinspires.ftc.teamcode.extensions

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import org.firstinspires.ftc.teamcode.Point
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.AutoHardware

object PoseExtensions {
    fun Pose2d.toPoint(): Point {
        return Point(this.position.x, this.position.y)
    }

    fun Vector2d.toPoint(): Point {
        return Point(this.x, this.y)
    }

    fun Pose2d.toStartPose(): AutoHardware.StartPose {
        return AutoHardware.StartPose(
            this.position.x,
            this.position.y,
            Math.toDegrees(this.heading.toDouble())
        )
    }

}