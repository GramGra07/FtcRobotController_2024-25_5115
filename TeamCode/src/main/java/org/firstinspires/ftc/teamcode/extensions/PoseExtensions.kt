package org.firstinspires.ftc.teamcode.extensions

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import org.firstinspires.ftc.teamcode.followers.pedroPathing.localization.Pose
import org.firstinspires.ftc.teamcode.utilClass.objects.Point
import kotlin.math.roundToInt

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
        return Pose2d(this.x, this.y, -this.heading)
    }

    fun Pose2d.toPose2D(): SparkFunOTOS.Pose2D {
        return SparkFunOTOS.Pose2D(
            this.position.x,
            this.position.y,
            Math.toDegrees(this.heading.toDouble())
        )
    }

    fun Pose2d.toString2(): String {
        return "x: ${this.position.x.roundToInt()}, y: ${this.position.y.roundToInt()}, h: ${
            Math.toDegrees(
                this.heading.toDouble()
            ).roundToInt()
        }"
    }

    fun Pose.toString2(): String {
        return "x: ${this.x.roundToInt()}, y: ${this.y.roundToInt()}, h: ${
            Math.toDegrees(
                this.heading.toDouble()
            ).roundToInt()
        }"
    }

    fun SparkFunOTOS.Pose2D.toPoint(): Point {
        return Point(this.x, this.y)
    }

    fun org.opencv.core.Point.toString(): String {
        return "x: ${this.x}, y: ${this.y}"
    }

    fun org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.Point.toPose2d(heading: Double): Pose2d {
        return Pose2d(this.x, this.y, heading)
    }

    fun org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.Point.toPose(heading: Double): Pose {
        return Pose(this.x, this.y, heading)
    }
}