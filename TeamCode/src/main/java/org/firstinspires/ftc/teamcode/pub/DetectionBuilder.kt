package org.firstinspires.ftc.teamcode.pub

import org.opencv.core.Rect

/**
 * This interface is used to build a detection object
 * @property rectangle the rectangle of the detection
 * @property name the name of the detection
 * @property function the function to execute when the detection is detected
 */
class DetectionBuilder(
    override val rectangle: Rect,
    override val name: String,
    private val function: () -> Unit
) : ObjectDetectionBuilder {
    override fun execute() {
        function.invoke()
    }
}