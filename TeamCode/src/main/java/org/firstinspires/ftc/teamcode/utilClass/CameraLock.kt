package org.firstinspires.ftc.teamcode.utilClass

import org.firstinspires.ftc.teamcode.customHardware.sensors.BrushlandRoboticsSensor
import org.firstinspires.ftc.teamcode.extensions.VisionExtensions.cross
import org.firstinspires.ftc.teamcode.utilClass.objects.BinaryArray
import org.firstinspires.ftc.teamcode.utilClass.objects.LLFormattedResult
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import kotlin.math.roundToInt

class CameraLock(
    var angle: Double,
    var color: BinaryArray,
    var center: Point,
) {
    companion object {
        fun empty(): CameraLock {
            return CameraLock(0.0, BinaryArray(1), Point(0.0, 0.0))
        }
    }
    fun draw(frame: Mat) {
        cross(frame, center, 10, Scalar(255.0, 0.0, 0.0))
        Imgproc.circle(frame, center, 10, Scalar(255.0, 0.0, 0.0), 1)
    }

    override fun toString(): String {
        return "Center: $center, Angle: $angle, Color: ${color.toColor().toString()}"
    }

    fun Point.toString(): String {
        return "${center.x.roundToInt()}, ${center.y.roundToInt()}"
    }
}