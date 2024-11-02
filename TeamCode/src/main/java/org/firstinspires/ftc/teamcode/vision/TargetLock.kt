package org.firstinspires.ftc.teamcode.vision

import android.graphics.Bitmap
import android.graphics.Canvas
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.function.Consumer
import org.firstinspires.ftc.robotcore.external.function.Continuation
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance
import org.firstinspires.ftc.teamcode.customHardware.sensors.BrushlandRoboticsSensor
import org.firstinspires.ftc.teamcode.utilClass.CameraLock
import org.firstinspires.ftc.teamcode.utilClass.ScalarPair
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.android.Utils
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import java.util.concurrent.atomic.AtomicReference
import kotlin.math.pow
import kotlin.math.sqrt

class TargetLock(
    private var alliance: Alliance,
    private var camOrientation: Double? = 0.0,
) : VisionProcessor,
    CameraStreamSource {
    private var sentAngle = camOrientation
    private val lastFrame = AtomicReference(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565))
    private var ycrcbMat = Mat()

    private var yellow = Mat()
    private var allianceColor = Mat()
    private var edgesA = Mat()
    private var edgesY = Mat()

    var current = 0
    private var minArea = 1600
    private var scalar: ScalarPair =
        ScalarPair(Scalar(0.0, 0.0, 130.0), Scalar(255.0, 255.0, 255.0))

    override fun init(width: Int, height: Int, calibration: CameraCalibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565))
    }

    override fun processFrame(frame: Mat, captureTimeNanos: Long): Any {
        if (alliance == Alliance.RED) {
            scalar.low = ConfigScalars.redLow
            scalar.high = ConfigScalars.redHigh
        } else {
            scalar.low = ConfigScalars.blueLow
            scalar.high = ConfigScalars.blueHigh
        }
        // Convert RGB to YCrCb color space
        Imgproc.cvtColor(frame, ycrcbMat, Imgproc.COLOR_BGR2YCrCb)

        Core.inRange(ycrcbMat, scalar.low, scalar.high, allianceColor)
        Core.inRange(ycrcbMat, ConfigScalars.yellowLow, ConfigScalars.yellowHigh, yellow)
        Imgproc.Canny(allianceColor, edgesA, 200.0, 255.0)
        var closestLockA =
            CameraLock(Point(0.0, 0.0), 0.0, BrushlandRoboticsSensor.Color.NONE, false)
        var closestDistance = Double.POSITIVE_INFINITY
        var largestA = 0

        val contours: List<MatOfPoint> = ArrayList()
        Imgproc.findContours(
            edgesA,
            contours,
            Mat(),
            Imgproc.RETR_TREE,
            Imgproc.CHAIN_APPROX_SIMPLE
        )

        val contoursPoly: Array<MatOfPoint2f?> = arrayOfNulls(contours.size)
        val centers = arrayOfNulls<Point>(contours.size)
        for (i in contours.indices) {
            // Approximate each contour to a polygon
            contoursPoly[i] = MatOfPoint2f()
            Imgproc.approxPolyDP(MatOfPoint2f(*contours[i].toArray()), contoursPoly[i], 3.0, true)

            // Create a rotated rectangle around the detected object
            val rotatedRect = Imgproc.minAreaRect(contoursPoly[i])
            val rectPoints = arrayOfNulls<Point>(4)
            rotatedRect.points(rectPoints)
            val center = rotatedRect.center
            if (rotatedRect.size.area() > minArea) {
                centers[i] = center
                // Draw the rotated rectangle
                for (j in 0 until 4) {
                    Imgproc.line(
                        frame,
                        rectPoints[j],
                        rectPoints[(j + 1) % 4],
                        scalar.low,
                        2
                    )
                }

                val dist =
                    sqrt(
                        (frame.width() / 2 - center.x).pow(2) + (frame.height() / 2 - center.y).pow(
                            2
                        )
                    )
                if (dist < closestDistance && rotatedRect.size.area() > minArea && rotatedRect.size.area() > largestA) {
                    largestA = rotatedRect.size.area().toInt()
                    closestDistance = dist
                    val color = if (alliance == Alliance.RED) {
                        BrushlandRoboticsSensor.Color.RED
                    } else {
                        BrushlandRoboticsSensor.Color.BLUE
                    }
                    val b = (largestA < 10000)
                    closestLockA =
                        CameraLock(
                            center,
                            rotatedRect.angle - (sentAngle ?: 0.0),
                            color,
                            b
                        )
                }
            }
        }

        Imgproc.Canny(yellow, edgesY, 200.0, 255.0)
        var closestLockY =
            CameraLock(Point(0.0, 0.0), 0.0, BrushlandRoboticsSensor.Color.NONE, false)
        var closestDistancey = Double.POSITIVE_INFINITY
        var largestY = 0

        val contoursy: List<MatOfPoint> = ArrayList()
        Imgproc.findContours(
            edgesY,
            contoursy,
            Mat(),
            Imgproc.RETR_TREE,
            Imgproc.CHAIN_APPROX_SIMPLE
        )

        val contoursPolyy: Array<MatOfPoint2f?> = arrayOfNulls(contoursy.size)
        val centersy = arrayOfNulls<Point>(contoursy.size)
        for (i in contoursy.indices) {
            contoursPolyy[i] = MatOfPoint2f()
            Imgproc.approxPolyDP(MatOfPoint2f(*contoursy[i].toArray()), contoursPolyy[i], 3.0, true)
            val rotatedRecty = Imgproc.minAreaRect(contoursPolyy[i])
            val rectPointsy = arrayOfNulls<Point>(4)
            rotatedRecty.points(rectPointsy)
            val center = rotatedRecty.center
            if (rotatedRecty.size.area() > minArea) {
                centersy[i] = center
                for (j in 0 until 4) {
                    Imgproc.line(
                        frame,
                        rectPointsy[j],
                        rectPointsy[(j + 1) % 4],
                        Scalar(210.0, 146.0, 16.0),
                        2
                    )
                }

                val dist =
                    sqrt(
                        (frame.width() / 2 - center.x).pow(2) + (frame.height() / 2 - center.y).pow(
                            2
                        )
                    )
                if (dist < closestDistancey && rotatedRecty.size.area() > minArea && rotatedRecty.size.area() > largestY) {
                    closestDistancey = dist
                    largestY = rotatedRecty.size.area().toInt()
                    val y = (largestY > 10000)
                    closestLockY =
                        CameraLock(
                            center,
                            rotatedRecty.angle - (sentAngle ?: 0.0),
                            BrushlandRoboticsSensor.Color.YELLOW, y
                        )
                }
            }
        }

        val dista =
            sqrt(
                (frame.width() / 2 - closestLockA.center.x).pow(2) + (frame.height() / 2 - closestLockA.center.y).pow(
                    2
                )
            )
        val disty =
            sqrt(
                (frame.width() / 2 - closestLockY.center.x).pow(2) + (frame.height() / 2 - closestLockY.center.y).pow(
                    2
                )
            )
        cameraLock = if (dista > disty) {
            closestLockY
        } else {
            closestLockA
        }
        cameraLock.draw(frame)
        edgesA.release()
        yellow.release()
        allianceColor.release()
        val b =
            Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565)
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

    fun sendAngle(angle: Double) {
        sentAngle = angle
    }

    companion object {
        var cameraLock: CameraLock =
            CameraLock(Point(0.0, 0.0), 0.0, BrushlandRoboticsSensor.Color.NONE, false)

        fun telemetry(telemetry: Telemetry) {
            telemetry.addData("Camera Lock", cameraLock.toString())
        }
    }
}
