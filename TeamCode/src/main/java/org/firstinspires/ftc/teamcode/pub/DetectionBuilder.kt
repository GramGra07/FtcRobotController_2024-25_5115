package org.firstinspires.ftc.teamcode.pub

import org.firstinspires.ftc.teamcode.pub.builderInterfaces.ObjectDetectionBuilder
import org.opencv.core.Rect
import org.opencv.core.Scalar

/**
 * This interface is used to build a detection object
 * @property rectangle the rectangle of the detection
 * @property name the name of the detection
 * @property function the function to execute when the detection is detected
 */
class DetectionBuilder(
    override val rectangle: Rect,
    override val name: String,
    override val scalarLow: Scalar,
    override val scalarHigh: Scalar,
    private val function: () -> Unit
) : ObjectDetectionBuilder {
    override fun execute() {
        function.invoke()
    }
}