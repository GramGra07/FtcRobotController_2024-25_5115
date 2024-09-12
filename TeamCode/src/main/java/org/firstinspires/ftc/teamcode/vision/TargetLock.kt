package org.firstinspires.ftc.teamcode.vision

import android.graphics.Bitmap
import android.graphics.Canvas
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.function.Consumer
import org.firstinspires.ftc.robotcore.external.function.Continuation
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.FastIntakeSubsystem
import org.firstinspires.ftc.teamcode.utilClass.CameraLock
import org.firstinspires.ftc.teamcode.utilClass.ScalarPair
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.android.Utils
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Point
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import java.util.concurrent.atomic.AtomicReference
import kotlin.math.pow
import kotlin.math.sqrt

class TargetLock(private var alliance: Alliance, private var fov:Double = 180.0) : VisionProcessor,
    CameraStreamSource {
    private val lastFrame = AtomicReference(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565))
    private var ycrcbMat = Mat()

    var yellow = Mat()
    var allianceColor = Mat()
    var detectionMat = Mat()
    var edges = Mat()

    //    var c = Scalar(255.0, 0.0, 0.0)
    var current = 0
    private lateinit var scalar: ScalarPair
    private val scalarYellowLow = Scalar(0.0, 140.0, 140.0)
    private val scalarYellowHigh = Scalar(255.0, 255.0, 255.0)

    override fun init(width: Int, height: Int, calibration: CameraCalibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565))
    }

    override fun processFrame(frame: Mat, captureTimeNanos: Long): Any {
        if (alliance == Alliance.RED) {
            scalar.low = Scalar(0.0, 140.0, 0.0)
            scalar.high = Scalar(255.0, 255.0, 255.0)
        } else if (alliance == Alliance.BLUE) {
            scalar.low = Scalar(0.0, 0.0, 130.0)
            scalar.high = Scalar(255.0, 255.0, 255.0)
        }
        Imgproc.cvtColor(frame, ycrcbMat, Imgproc.COLOR_RGB2YCrCb)

        Core.inRange(ycrcbMat, scalar.low, scalar.high, allianceColor)
        Core.inRange(ycrcbMat, scalarYellowLow, scalarYellowHigh, yellow)

        ycrcbMat.release()

        Core.bitwise_or(allianceColor, yellow, detectionMat)
        Imgproc.Canny(detectionMat, edges, 50.0, 150.0)

        detectionMat.release()

        val contours: List<MatOfPoint> = ArrayList<MatOfPoint>()
        Imgproc.findContours(
            edges,
            contours,
            Mat(),
            Imgproc.RETR_TREE,
            Imgproc.CHAIN_APPROX_SIMPLE
        )
        val contoursPoly: Array<MatOfPoint2f?> = arrayOfNulls<MatOfPoint2f>(contours.size)
        val boundRect = arrayOfNulls<Rect>(contours.size)
        val centers = arrayOfNulls<Point>(contours.size)
        val radius = Array(contours.size) { FloatArray(1) }
        for (i in contours.indices) {
            contoursPoly[i] = MatOfPoint2f()
            Imgproc.approxPolyDP(
                MatOfPoint2f(*contours[i].toArray()),
                contoursPoly[i],
                3.0,
                true
            )
            boundRect[i] = Imgproc.boundingRect(MatOfPoint(*contoursPoly[i]?.toArray()))
            centers[i] = Point()
            Imgproc.minEnclosingCircle(contoursPoly[i], centers[i], radius[i])
        }
        val contoursPolyList: MutableList<MatOfPoint> = ArrayList(contoursPoly.size)
        for (poly in contoursPoly) {
            contoursPolyList.add(MatOfPoint(*poly?.toArray()))
        }
        for (i in contours.indices) {
            val c: Scalar = scalar.low
            Imgproc.drawContours(frame, contoursPolyList, i, c)
            Imgproc.rectangle(frame, boundRect[i]!!.tl(), boundRect[i]!!.br(), c, 2)
        }

        var closestLock = CameraLock(FastIntakeSubsystem.Color.NONE,Point(0.0,0.0),0.0)
        var closestDistance = Double.POSITIVE_INFINITY
        for (center in centers) {
            val dist = sqrt(
                ((frame.width() / 2 - center!!.x).pow(2)) + (frame.height() / 2 - center.y).pow(2)
            )
            if (dist < closestDistance) {
                closestDistance = dist
                val angle = (fov/frame.width())*center.x
                closestLock = CameraLock(FastIntakeSubsystem.Color.YELLOW,Point(center.x, center.y),angle)
            }
        }
        cameraLock = closestLock
        cameraLock.draw(frame)

        edges.release()
        yellow.release()
        allianceColor.release()
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
        var cameraLock:CameraLock = CameraLock(FastIntakeSubsystem.Color.NONE,Point(0.0,0.0),0.0)
        fun telemetry(telemetry: Telemetry){
            telemetry.addData("Camera Lock", cameraLock.toString())
        }
    }
}
