package org.firstinspires.ftc.teamcode.utilClass

import org.firstinspires.ftc.teamcode.extensions.VisionExtensions.cross
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import kotlin.math.roundToInt

class CameraLock(var center: Point, var angle: Double) {
    fun draw(frame: Mat) {
        cross(frame, center, 10, Scalar(255.0, 0.0, 0.0))
        Imgproc.circle(frame, center, 10, Scalar(255.0, 0.0, 0.0), 1)
    }

    override fun toString(): String {
        return "Center: $center, Angle: $angle"
    }

    fun Point.toString(): String {
        return "${center.x.roundToInt()}, ${center.y.roundToInt()}"
    }
}