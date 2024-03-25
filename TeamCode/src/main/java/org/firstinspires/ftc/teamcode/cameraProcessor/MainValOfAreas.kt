package org.firstinspires.ftc.teamcode.cameraProcessor

import android.graphics.Bitmap
import android.graphics.Canvas
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import org.firstinspires.ftc.robotcore.external.function.Consumer
import org.firstinspires.ftc.robotcore.external.function.Continuation
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.teamcode.extensions.BlinkExtensions.setPatternCo
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.lights
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.android.Utils
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import java.util.concurrent.atomic.AtomicReference

class MainValOfAreas(
    scalarLow: Scalar,
    scalarHigh: Scalar,
    rectangles: MutableList<Rect>,
    firstName: String,
    secondName: String,
    defaultName: String
) : VisionProcessor, CameraStreamSource {

    private val lastFrame = AtomicReference(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565))
    private var ycrcbMat = Mat()
    private var right = Mat()
    private var middle = Mat()
    private var current = 0

    private var scalarLow: Scalar
    private var scalarHigh: Scalar
    private var rectangles: MutableList<Rect>
    private var firstName: String
    private var secondName: String
    private var defaultName: String

    init {
        this.scalarLow = scalarLow
        this.scalarHigh = scalarHigh
        this.rectangles = rectangles
        this.firstName = firstName
        this.secondName = secondName
        this.defaultName = defaultName
    }

    override fun init(width: Int, height: Int, calibration: CameraCalibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565))
    }

    override fun processFrame(frame: Mat, captureTimeNanos: Long): Any {
        if (rectangles.size > 2) {
            throw IllegalArgumentException("Too many rectangles requested")
        }

        for (rect in rectangles) {
            Imgproc.rectangle(frame, rect.tl(), rect.br(), Scalar(0.0, 255.0, 0.0), 1)
        }

        Imgproc.cvtColor(frame, ycrcbMat, Imgproc.COLOR_RGB2YCrCb)
        middle = ycrcbMat.submat(rectangles[0])
        right = ycrcbMat.submat(rectangles[1])

        val rightMean = Core.mean(right).`val`
        val middleMean = Core.mean(middle).`val`

        if (checkWithinBounds(rightMean)) {
            Imgproc.rectangle(
                frame,
                rectangles[0].br(),
                rectangles[0].tl(),
                Scalar(0.0, 255.0, 0.0),
                1
            )
            Imgproc.putText(
                frame, firstName,
                Point((frame.width() / 2).toDouble(), (frame.height() / 2).toDouble()),
                0, 5.0, Scalar(0.0, 255.0, 0.0)
            )
            current = 1
        } else if (current != 1 && checkWithinBounds(middleMean)) {
            Imgproc.rectangle(
                frame,
                rectangles[1].br(),
                rectangles[1].tl(),
                Scalar(0.0, 255.0, 0.0),
                1
            )
            Imgproc.putText(
                frame, secondName,
                Point((frame.width() / 2).toDouble(), (frame.height() / 2).toDouble()),
                0, 5.0, Scalar(0.0, 255.0, 0.0)
            )
            current = 1
        } else if (current == 0) {
            Imgproc.putText(
                frame, defaultName,
                Point((frame.width() / 2).toDouble(), (frame.height() / 2).toDouble()),
                0, 5.0, Scalar(0.0, 255.0, 0.0)
            )
            lights.setPatternCo(RevBlinkinLedDriver.BlinkinPattern.CONFETTI)
        }

        current = 0
        ycrcbMat.release()
        right.release()
        middle.release()

        val b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565)
        Utils.matToBitmap(frame, b)
        lastFrame.set(b)
        return frame
    }

    private fun checkWithinBounds(mean: DoubleArray): Boolean {
        return (mean[0] > scalarLow.`val`[0] && mean[0] < scalarHigh.`val`[0] &&
                mean[1] > scalarLow.`val`[1] && mean[1] < scalarHigh.`val`[1] &&
                mean[2] > scalarLow.`val`[2] && mean[2] < scalarHigh.`val`[2])
    }

    override fun onDrawFrame(
        canvas: Canvas,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any
    ) {
        // Not implemented
    }

    override fun getFrameBitmap(continuation: Continuation<out Consumer<Bitmap>?>) {
        continuation.dispatch { bitmapConsumer: Consumer<Bitmap>? ->
            bitmapConsumer!!.accept(lastFrame.get())
        }
    }
}
