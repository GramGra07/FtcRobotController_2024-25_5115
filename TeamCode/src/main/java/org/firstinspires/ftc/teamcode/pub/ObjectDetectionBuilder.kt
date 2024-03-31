package org.firstinspires.ftc.teamcode.pub

import org.opencv.core.Rect

interface ObjectDetectionBuilder {
    val rectangle: Rect
    val name: String
    fun execute()
}