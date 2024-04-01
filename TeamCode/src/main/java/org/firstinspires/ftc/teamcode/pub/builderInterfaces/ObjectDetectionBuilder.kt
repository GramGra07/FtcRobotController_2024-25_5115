package org.firstinspires.ftc.teamcode.pub.builderInterfaces

import org.opencv.core.Rect
import org.opencv.core.Scalar

interface ObjectDetectionBuilder {
    val rectangle: Rect
    val name: String
    val scalarLow: Scalar
    val scalarHigh: Scalar
    fun execute()
}