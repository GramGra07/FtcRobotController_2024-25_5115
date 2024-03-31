package org.firstinspires.ftc.teamcode.pub

import org.opencv.core.Rect

/**
 * A builder for an assumed detection.
 * @param name The name of the detection.
 * @param function The function to execute when the detection is assumed.
 * @param rectangle **IGNORE**
 */
class AssumedDetectionBuilder(
    override val name: String,
    private val function: () -> Unit,
    override val rectangle: Rect = Rect(),
) : ObjectDetectionBuilder {
    override fun execute() {
        function.invoke()
    }
}