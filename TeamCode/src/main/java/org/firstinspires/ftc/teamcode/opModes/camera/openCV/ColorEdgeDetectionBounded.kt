package org.firstinspires.ftc.teamcode.opModes.camera.openCV

import org.firstinspires.ftc.teamcode.Enums.Alliance
import org.firstinspires.ftc.teamcode.Enums.AutoRandom
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Point
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline

//@Config
class ColorEdgeDetectionBounded(var alliance: Alliance) : OpenCvPipeline() {
    var end = Mat()
    var edges = Mat()
    var hierarchy = Mat()
    var ycrcbMat = Mat()
    var c = Scalar(255.0, 0.0, 0.0)
    var scalarLow: Scalar? = null
    var scalarHigh: Scalar? = null
    override fun processFrame(input: Mat): Mat {
//        EOCVWebcam.pipelineName = "Color Edge Detection Bounded";
        // color map below
        // https://i.stack.imgur.com/gyuw4.png
        if (alliance == Alliance.RED) {
            scalarLow = Scalar(0.0, 147.0, 0.0)
            scalarHigh = Scalar(255.0, 255.0, 255.0)
        } else if (alliance == Alliance.BLUE) {
            //todo change to blue
            scalarLow = Scalar(0.0, 0.0, 141.0)
            scalarHigh = Scalar(255.0, 255.0, 255.0)
        }
        Imgproc.rectangle(
            input, Point(pointsX[0].toDouble(), pointsY[0].toDouble()), Point(
                pointsX[1].toDouble(), pointsY[1].toDouble()
            ), Scalar(0.0, 255.0, 0.0), 1
        )
        Imgproc.rectangle(
            input, Point(pointsX[2].toDouble(), pointsY[2].toDouble()), Point(
                pointsX[3].toDouble(), pointsY[3].toDouble()
            ), Scalar(0.0, 255.0, 0.0), 1
        )
        Imgproc.rectangle(
            input, Point(pointsX[4].toDouble(), pointsY[4].toDouble()), Point(
                pointsX[5].toDouble(), pointsY[5].toDouble()
            ), Scalar(0.0, 255.0, 0.0), 1
        )
        Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb) //change to hsv
        Core.inRange(ycrcbMat, scalarLow, scalarHigh, end)
        Core.bitwise_and(input, input, ycrcbMat, end)
        Imgproc.Canny(end, edges, 25.0, 50.0)
        val contours: List<MatOfPoint> = ArrayList()
        Imgproc.findContours(
            edges,
            contours,
            hierarchy,
            Imgproc.RETR_TREE,
            Imgproc.CHAIN_APPROX_SIMPLE
        )
        val contoursPoly = arrayOfNulls<MatOfPoint2f>(contours.size)
        val boundRect = arrayOfNulls<Rect>(contours.size)
        val centers = arrayOfNulls<Point>(contours.size)
        val radius = Array(contours.size) { FloatArray(1) }
        for (i in contours.indices) {
            contoursPoly[i] = MatOfPoint2f()
            Imgproc.approxPolyDP(MatOfPoint2f(*contours[i].toArray()), contoursPoly[i], 3.0, true)
            centers[i] = Point()
            boundRect[i] = Imgproc.boundingRect(MatOfPoint(*contoursPoly[i]!!.toArray()))
            Imgproc.minEnclosingCircle(contoursPoly[i], centers[i], radius[i])
        }
        val contoursPolyList: MutableList<MatOfPoint> = ArrayList(contoursPoly.size)
        for (poly in contoursPoly) {
            contoursPolyList.add(MatOfPoint(*poly!!.toArray()))
        }
        var highIndex = 0
        for (i in contours.indices) {
            if (centers[i]!!.x > pointsX[0] && centers[i]!!.x < pointsX[1] && centers[i]!!.y > pointsY[0] && centers[i]!!.y < pointsY[1] || centers[i]!!.x > pointsX[2] && centers[i]!!.x < pointsX[3] && centers[i]!!.y > pointsY[2] && centers[i]!!.y < pointsY[3] || centers[i]!!.x > pointsX[4] && centers[i]!!.x < pointsX[5] && centers[i]!!.y > pointsY[4] && centers[i]!!.y < pointsY[5]
            ) {
                Imgproc.drawContours(input, contoursPolyList, i, c)
                Imgproc.rectangle(input, boundRect[i]!!.tl(), boundRect[i]!!.br(), c, 1)
                val centerX = boundRect[i]!!.tl().x + (boundRect[i]!!
                    .br().x - boundRect[i]!!.tl().x) / 2
                val centerY = boundRect[i]!!.br().y + (boundRect[i]!!
                    .tl().y - boundRect[i]!!.br().y) / 2
                Imgproc.line(
                    input,
                    Point(centerX - 5, centerY),
                    Point(centerX + 5, centerY),
                    Scalar(0.0, 255.0, 200.0)
                )
                Imgproc.line(
                    input,
                    Point(centerX, centerY - 5),
                    Point(centerX, centerY + 5),
                    Scalar(0.0, 255.0, 200.0)
                )
                if (boundRect[i]!!.height > boundRect[highIndex]!!.height && boundRect[i]!!.width > boundRect[highIndex]!!.width) //get largest rectangle
                    highIndex = i
            }
        }
        // check which side it is actually on using centers
        if (highIndex != 0) {
            if (centers[highIndex]!!.x > pointsX[0] && centers[highIndex]!!.x < pointsX[1]) {
                if (centers[highIndex]!!.y > pointsY[0] && centers[highIndex]!!.y < pointsY[1]) {
                    Imgproc.rectangle(
                        input, Point(pointsX[0].toDouble(), pointsY[0].toDouble()), Point(
                            pointsX[1].toDouble(), pointsY[1].toDouble()
                        ), Scalar(0.0, 255.0, 0.0), 1
                    )
                    Imgproc.putText(
                        input,
                        "middle",
                        Point(pointsX[0].toDouble(), pointsY[0].toDouble()),
                        0,
                        1.0,
                        Scalar(0.0, 255.0, 0.0)
                    )
                    autoHardware.autonomousRandom = AutoRandom.left
                }
            }
            if (centers[highIndex]!!.x > pointsX[2] && centers[highIndex]!!.x < pointsX[3]) {
                if (centers[highIndex]!!.y > pointsY[2] && centers[highIndex]!!.y < pointsY[3]) {
                    Imgproc.rectangle(
                        input, Point(pointsX[2].toDouble(), pointsY[2].toDouble()), Point(
                            pointsX[3].toDouble(), pointsY[3].toDouble()
                        ), Scalar(0.0, 255.0, 0.0), 1
                    )
                    Imgproc.putText(
                        input,
                        "center",
                        Point(pointsX[2].toDouble(), pointsY[2].toDouble()),
                        0,
                        1.0,
                        Scalar(0.0, 255.0, 0.0)
                    )
                    autoHardware.autonomousRandom = AutoRandom.mid
                }
            }
            if (centers[highIndex]!!.x > pointsX[4] && centers[highIndex]!!.x < pointsX[5]) {
                if (centers[highIndex]!!.y > pointsY[4] && centers[highIndex]!!.y < pointsY[5]) {
                    Imgproc.rectangle(
                        input, Point(pointsX[4].toDouble(), pointsY[4].toDouble()), Point(
                            pointsX[5].toDouble(), pointsY[5].toDouble()
                        ), Scalar(0.0, 255.0, 0.0), 1
                    )
                    Imgproc.putText(
                        input,
                        "right",
                        Point(pointsX[4].toDouble(), pointsY[4].toDouble()),
                        0,
                        1.0,
                        Scalar(0.0, 255.0, 0.0)
                    )
                    autoHardware.autonomousRandom = AutoRandom.right
                }
            }
        }
        return input
    }

    companion object {
        var pointsX = intArrayOf(0, 35, 75, 250, 270, 320)
        var pointsY = intArrayOf(160, 260, 170, 210, 180, 220)
    }
}