package org.firstinspires.ftc.teamcode.vision

import android.graphics.Bitmap
import android.graphics.Canvas
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import org.firstinspires.ftc.robotcore.external.function.Consumer
import org.firstinspires.ftc.robotcore.external.function.Continuation
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig.Companion.lights
import org.firstinspires.ftc.teamcode.extensions.BlinkExtensions.setPatternCo
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.android.Utils
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import java.util.concurrent.atomic.AtomicReference

class VPObjectDetect : VisionProcessor, CameraStreamSource { //var alliance: Alliance
    private val lastFrame = AtomicReference(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565))
    private var ycrcbMat = Mat()
    var right = Mat()
    private var middle = Mat()
    var c = Scalar(255.0, 0.0, 0.0)
    var current = 0
    private lateinit var scalarLow: Scalar
    private lateinit var scalarHigh: Scalar
    override fun init(width: Int, height: Int, calibration: CameraCalibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565))
    }

    override fun processFrame(frame: Mat, captureTimeNanos: Long): Any {
//        if (alliance == Alliance.RED) {
        scalarLow = Scalar(0.0, 140.0, 0.0)
        scalarHigh = Scalar(255.0, 255.0, 255.0)
//        } else if (alliance == Alliance.BLUE) {
//            scalarLow = Scalar(0.0, 0.0, 130.0)
//            scalarHigh = Scalar(255.0, 255.0, 255.0)
//        }
        Imgproc.rectangle(
            frame, Point(pointsX[0].toDouble(), pointsY[0].toDouble()), Point(
                pointsX[1].toDouble(), pointsY[1].toDouble()
            ), Scalar(0.0, 255.0, 0.0), 1
        )
        Imgproc.rectangle(
            frame, Point(pointsX[2].toDouble(), pointsY[2].toDouble()), Point(
                pointsX[3].toDouble(), pointsY[3].toDouble()
            ), Scalar(0.0, 255.0, 0.0), 1
        )
        Imgproc.cvtColor(frame, ycrcbMat, Imgproc.COLOR_RGB2YCrCb)
        middle = ycrcbMat.submat(
            Rect(
                pointsX[0],
                pointsY[0],
                pointsX[1] - pointsX[0],
                pointsY[1] - pointsY[0]
            )
        )
        right = ycrcbMat.submat(
            Rect(
                pointsX[2],
                pointsY[2],
                pointsX[3] - pointsX[2],
                pointsY[3] - pointsY[2]
            )
        )
        Core.mean(right)
        Core.mean(middle)
        val rightMean = Core.mean(right).`val`
        val middleMean = Core.mean(middle).`val`
        // check if it is within the scalar low and high
        if (rightMean[0] > scalarLow.`val`[0] && rightMean[0] < scalarHigh.`val`[0]) {
            if (rightMean[1] > scalarLow.`val`[1] && rightMean[1] < scalarHigh.`val`[1]) {
                if (rightMean[2] > scalarLow.`val`[2] && rightMean[2] < scalarHigh.`val`[2]) {
                    Imgproc.rectangle(
                        frame, Point(pointsX[2].toDouble(), pointsY[2].toDouble()), Point(
                            pointsX[3].toDouble(), pointsY[3].toDouble()
                        ), Scalar(0.0, 255.0, 0.0), 1
                    )
                    Imgproc.putText(
                        frame,
                        "right",
                        Point((frame.width() / 2).toDouble(), (frame.height() / 2).toDouble()),
                        0,
                        5.0,
                        Scalar(0.0, 255.0, 0.0)
                    )
//                    AutoHardware.autonomousRandom = AutoRandom.RIGHT
                    lights.setPatternCo(RevBlinkinLedDriver.BlinkinPattern.WHITE)
                    current = 1
                }
            }
        }
        if (current != 1) {
            if (middleMean[0] > scalarLow.`val`[0] && middleMean[0] < scalarHigh.`val`[0]) {
                if (middleMean[1] > scalarLow.`val`[1] && middleMean[1] < scalarHigh.`val`[1]) {
                    if (middleMean[2] > scalarLow.`val`[2] && middleMean[2] < scalarHigh.`val`[2]) {
                        Imgproc.rectangle(
                            frame, Point(pointsX[0].toDouble(), pointsY[0].toDouble()), Point(
                                pointsX[1].toDouble(), pointsY[1].toDouble()
                            ), Scalar(0.0, 255.0, 0.0), 1
                        )
                        Imgproc.putText(
                            frame,
                            "middle",
                            Point((frame.width() / 2).toDouble(), (frame.height() / 2).toDouble()),
                            0,
                            5.0,
                            Scalar(0.0, 255.0, 0.0)
                        )
//                        AutoHardware.autonomousRandom = AutoRandom.MID
                        lights.setPatternCo(RevBlinkinLedDriver.BlinkinPattern.GOLD)
                        current = 1
                        //                        shiftOffset = 0;
                    }
                }
            }
        }
        if (current == 0) {
            Imgproc.putText(
                frame,
                "left",
                Point((frame.width() / 2).toDouble(), (frame.height() / 2).toDouble()),
                0,
                5.0,
                Scalar(0.0, 255.0, 0.0)
            )
//            AutoHardware.autonomousRandom = AutoRandom.LEFT
            lights.setPatternCo(RevBlinkinLedDriver.BlinkinPattern.CONFETTI)
            //            shiftOffset = -leftShift;
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

    override fun onDrawFrame(
        canvas: Canvas,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any
    ) {
    }

    override fun getFrameBitmap(continuation: Continuation<out Consumer<Bitmap>?>) {
        continuation.dispatch { bitmapConsumer: Consumer<Bitmap>? ->
            bitmapConsumer!!.accept(
                lastFrame.get()
            )
        }
    }

    companion object {
        var pointsX = intArrayOf(570, 680, 120, 230)
        var pointsY = intArrayOf(70, 170, 50, 150)
    }
}
