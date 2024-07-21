package org.firstinspires.ftc.teamcode.extensions

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import org.firstinspires.ftc.teamcode.followers.pedroPathing.localization.Pose
import org.firstinspires.ftc.teamcode.utilClass.objects.Point

object PoseExtensions {
    fun Pose2d.toPose(): Pose {
        return Pose(this.position.x, this.position.y, this.heading.toDouble())
    }

    fun Pose2d.toPoint(): Point {
        return Point(this.position.x, this.position.y)
    }

    fun Vector2d.toPoint(): Point {
        return Point(this.x, this.y)
    }

    fun Point.distanceTo(point: Point): Double {
        val dx = point.x!! - this.x!!
        val dy = point.y!! - this.y!!
        return kotlin.math.sqrt(dx * dx + dy * dy)
    }

    fun Pose.toPose2d(): Pose2d {
        return Pose2d(this.x, this.y, this.heading)
    }
}