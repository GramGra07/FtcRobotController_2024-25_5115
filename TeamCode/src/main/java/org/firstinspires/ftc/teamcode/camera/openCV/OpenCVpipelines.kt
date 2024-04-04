package org.firstinspires.ftc.teamcode.camera.openCV

import com.acmerobotics.dashboard.FtcDashboard
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.UtilClass.varConfigurations.varConfig
import org.firstinspires.ftc.teamcode.camera.openCV.trainer.vars.blueconeObjVars
import org.firstinspires.ftc.teamcode.camera.openCV.trainer.vars.redconeObjVars
import org.firstinspires.ftc.teamcode.camera.openCV.trainer.vars.redpropObjVars
import org.firstinspires.ftc.teamcode.extensions.ScalarUtil.fetchScalar
import org.firstinspires.ftc.teamcode.extensions.ScalarUtil.scalarVals
import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Point
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline

class OpenCVpipelines {
    class EdgeDetection : OpenCvPipeline() {
        var gray: Mat = Mat()
        var edges: Mat = Mat()
        override fun processFrame(input: Mat): Mat {
//            EOCVWebcam.pipelineName = "Edge Detection";
            Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY)
            Imgproc.Canny(gray, edges, 50.0, 100.0)
            return edges
        }
    }

    class ColorDetection(var color: String) : OpenCvPipeline() {
        //isolation of color
        var ycrcbMat: Mat = Mat()
        var mask1: Mat = Mat()
        var mask2: Mat = Mat()
        var ycrcbMat2: Mat = Mat()
        var end: Mat = Mat()
        override fun processFrame(input: Mat): Mat {
//            EOCVWebcam.pipelineName = "Color Detection";
            val scalarLow: Scalar
            val scalarHigh: Scalar
            Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb) //change to ycrcbMat
            if (color != "red") {
                scalarLow = fetchScalar("l", color, 0)
                scalarHigh = fetchScalar("h", color, 0)
                Core.inRange(ycrcbMat, scalarLow, scalarHigh, end) //detect color, output to end
            } else {
                Core.inRange(
                    ycrcbMat,
                    fetchScalar("l", color, 1),
                    fetchScalar("h", color, 1),
                    mask1
                )
                Core.inRange(
                    ycrcbMat2,
                    fetchScalar("l", color, 2),
                    fetchScalar("h", color, 2),
                    mask2
                )
                Core.bitwise_or(mask1, mask2, end) //takes both masks and combines them
            }
            return end
        }
    }

    class ColorEdgeDetection(var color: String) : OpenCvPipeline() {
        var edges: Mat = Mat()
        var ycrcbMat: Mat = Mat()
        var mask1: Mat = Mat()
        var ycrcbMat2: Mat = Mat()
        var mask2: Mat = Mat()
        var end: Mat = Mat()
        override fun processFrame(input: Mat): Mat {

//            EOCVWebcam.pipelineName = "Color Edge Detection";
            // color map below
            // https://i.stack.imgur.com/gyuw4.png
            val scalarLow: Scalar
            val scalarHigh: Scalar
            Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb) //change to ycrcbMat
            if (color != "red") {
                scalarLow = fetchScalar("l", color, 0)
                scalarHigh = fetchScalar("h", color, 0)
                Core.inRange(ycrcbMat, scalarLow, scalarHigh, end) //detect color, output to end
            } else {
                Core.inRange(
                    ycrcbMat,
                    fetchScalar("l", color, 1),
                    fetchScalar("h", color, 1),
                    mask1
                )
                Core.inRange(
                    ycrcbMat2,
                    fetchScalar("l", color, 2),
                    fetchScalar("h", color, 2),
                    mask2
                )
                Core.bitwise_or(mask1, mask2, end) //takes both masks and combines them
            }
            Imgproc.Canny(end, edges, 25.0, 50.0)
            return edges
        }
    }

    class ColorEdgeDetectionBounded(var color: String) : OpenCvPipeline() {
        var end: Mat = Mat()
        var edges: Mat = Mat()
        var hierarchy: Mat = Mat()
        var ycrcbMat: Mat = Mat()
        var ycrcbMat2: Mat = Mat()
        var mask1: Mat = Mat()
        var mask2: Mat = Mat()
        override fun processFrame(input: Mat): Mat {
//            EOCVWebcam.pipelineName = "Color Edge Detection Bounded";
            // color map below
            // https://i.stack.imgur.com/gyuw4.png
            val scalarLow: Scalar
            val scalarHigh: Scalar
            Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb) //change to ycrcbMat
            Imgproc.cvtColor(input, ycrcbMat2, Imgproc.COLOR_RGB2YCrCb)
            if (color != "red") {
                scalarLow = fetchScalar("l", color, 0)
                scalarHigh = fetchScalar("h", color, 0)
                Core.inRange(ycrcbMat, scalarLow, scalarHigh, end) //detect color, output to end
            } else {
                Core.inRange(
                    ycrcbMat,
                    fetchScalar("l", color, 1),
                    fetchScalar("h", color, 1),
                    mask1
                )
                Core.inRange(
                    ycrcbMat2,
                    fetchScalar("l", color, 2),
                    fetchScalar("h", color, 2),
                    mask2
                )
                Core.bitwise_or(mask1, mask2, end) //takes both masks and combines them
            }
            Imgproc.Canny(end, edges, 25.0, 50.0)
            val contours: List<MatOfPoint> = ArrayList<MatOfPoint>()
            Imgproc.findContours(
                edges,
                contours,
                hierarchy,
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
            val contoursPolyList: MutableList<MatOfPoint> = ArrayList<MatOfPoint>(contoursPoly.size)
            for (poly in contoursPoly) {
                contoursPolyList.add(MatOfPoint(*poly?.toArray()))
            }
            var highIndex = 0
            for (i in contours.indices) {
                val c: Scalar = scalarVals(color)
                Imgproc.drawContours(input, contoursPolyList, i, c)
                Imgproc.rectangle(input, boundRect[i]!!.tl(), boundRect[i]!!.br(), c, 2)
                if (boundRect[i]!!.height > boundRect[highIndex]!!.height && boundRect[i]!!.width > boundRect[highIndex]!!.width) //get largest rectangle
                    highIndex = i
            }
            if (boundRect.size > 0) {
                val left = boundRect[highIndex]!!.tl().x
                val right = boundRect[highIndex]!!.br().x
                val top = boundRect[highIndex]!!.tl().y
                val bottom = boundRect[highIndex]!!.br().y
                val centerX = (left + (right - left) / 2).toInt()
                Imgproc.putText(
                    input,
                    centerX.toString(),
                    Point(left + 7, top - 10),
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    scalarVals(
                        color
                    ),
                    2
                )
            }
            return input
        }
    }

    class WhiteDotDetection : OpenCvPipeline() {
        var blur: Mat = Mat()
        var gray: Mat = Mat()
        var thresh: Mat = Mat()
        var hierarchy: Mat = Mat()
        var min_area = 0.1
        override fun processFrame(input: Mat): Mat {
//            EOCVWebcam.pipelineName = "White Dot Detection";
            Imgproc.medianBlur(input, blur, 5)
            Imgproc.cvtColor(blur, gray, Imgproc.COLOR_BGR2GRAY)
            Imgproc.threshold(gray, thresh, 200.0, 255.0, Imgproc.THRESH_BINARY)
            val contours: List<MatOfPoint> = ArrayList<MatOfPoint>()
            val white_dots: MutableList<Any> = ArrayList()
            Imgproc.findContours(
                thresh,
                contours,
                hierarchy,
                Imgproc.RETR_EXTERNAL,
                Imgproc.CHAIN_APPROX_SIMPLE
            )
            for (c in contours) {
                val area: Double = Imgproc.contourArea(c as Mat)
                if (area > min_area) {
                    Imgproc.drawContours(input, contours, -1, scalarVals("green"), 2)
                    white_dots.add(c)
                }
            }
            //            EOCVWebcam.whiteDots = white_dots.size();
            Imgproc.putText(
                input,
                white_dots.size.toString(),
                Point((input.width() / 16).toDouble(), (input.height() / 6).toDouble()),
                Imgproc.FONT_HERSHEY_SIMPLEX,
                1.0,
                scalarVals("green"),
                2
            )
            return input
        }
    }

    class BlackDotDetection : OpenCvPipeline() {
        var gray: Mat = Mat()
        var circles: Mat = Mat()
        var thresh: Mat = Mat()
        var masked: Mat = Mat()
        override fun processFrame(input: Mat): Mat {
//            EOCVWebcam.pipelineName = "Black Dot Detection";
            Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY)
            Imgproc.medianBlur(gray, gray, 5)
            Imgproc.HoughCircles(
                gray, circles, Imgproc.HOUGH_GRADIENT, 1.0,
                0.1,
                100.0, 30.0, 1, 100
            )
            val mask = Mat(input.rows(), input.cols(), CvType.CV_8U, Scalar.all(0.0))
            if (circles.cols() > 0) {
                for (x in 0 until circles.cols()) {
                    val c: DoubleArray = circles.get(0, x)
                    val center = Point(
                        Math.round(c[0]).toDouble(), Math.round(
                            c[1]
                        ).toDouble()
                    )
                    //                    EOCVWebcam.blackDotCenterX = center.x;
//                    EOCVWebcam.blackDotCenterY = center.y;
                    val radius = Math.round(c[2]).toInt()
                    Imgproc.circle(mask, center, radius, Scalar(255.0, 255.0, 255.0), -1, 8, 0)
                }
            }
            input.copyTo(masked, mask)
            Imgproc.threshold(mask, thresh, 1.0, 255.0, Imgproc.THRESH_BINARY)
            val contours: List<MatOfPoint> = ArrayList<MatOfPoint>()
            Imgproc.findContours(
                thresh,
                contours,
                Mat(),
                Imgproc.RETR_EXTERNAL,
                Imgproc.CHAIN_APPROX_SIMPLE
            )
            if (contours.size > 0) {
                for (c in contours.indices) {
                    Imgproc.rectangle(
                        input, Imgproc.boundingRect(contours[c]).tl(), Imgproc.boundingRect(
                            contours[c]
                        ).br(), scalarVals("green"), 2
                    )
                }
            }
            //            EOCVWebcam.blackDots = contours.size();
            Imgproc.putText(
                input,
                contours.size.toString(),
                Point((input.width() / 16).toDouble(), (input.height() / 6).toDouble()),
                Imgproc.FONT_HERSHEY_SIMPLEX,
                1.0,
                scalarVals("green"),
                2
            )
            return input
        }
    }

    class RecognizeObject(var color: String, var obj: String) : OpenCvPipeline() {
        var name: String
        var aspectRatio = 0.0
        var xTranslation = 0.0
        var yTranslation = 0.0
        var tolerance = 0.0
        var minWidth = 0.0
        var minHeight = 0.0
        var minArea = 0.0
        var maxWidth = 0.0
        var maxHeight = 0.0
        var maxArea = 0.0
        var left = 0.0
        var right = 0.0
        var top = 0.0
        var bottom = 0.0
        var centerX = 0
        var ycrcbMat: Mat = Mat()
        var ycrcbMat2: Mat = Mat()
        var mask1: Mat = Mat()
        var mask2: Mat = Mat()
        var end: Mat = Mat()
        var edges: Mat = Mat()
        var hierarchy: Mat = Mat()

        init { // both uncaps
            name = color + obj
            aspectRatio = when (name) {
                "redcone" -> redconeObjVars.aspectRatio
                "bluecone" -> blueconeObjVars.aspectRatio
                "redprop" -> redpropObjVars.aspectRatio
                else -> 0.0
            }
        }

        override fun processFrame(input: Mat): Mat {
//            EOCVWebcam.pipelineName = "Recognize Object";
            val scalarLow: Scalar
            val scalarHigh: Scalar
            Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb) //change to ycrcbMat
            Imgproc.cvtColor(input, ycrcbMat2, Imgproc.COLOR_RGB2YCrCb)
            if (color != "red") {
                scalarLow = fetchScalar("l", color, 0)
                scalarHigh = fetchScalar("h", color, 0)
                Core.inRange(ycrcbMat, scalarLow, scalarHigh, end) //detect color, output to end
            } else {
                Core.inRange(
                    ycrcbMat,
                    fetchScalar("l", color, 1),
                    fetchScalar("h", color, 1),
                    mask1
                )
                Core.inRange(
                    ycrcbMat2,
                    fetchScalar("l", color, 2),
                    fetchScalar("h", color, 2),
                    mask2
                )
                Core.bitwise_or(mask1, mask2, end) //takes both masks and combines them
            }
            Imgproc.Canny(end, edges, 25.0, 50.0)
            val contours: List<MatOfPoint> = ArrayList<MatOfPoint>()
            Imgproc.findContours(
                edges,
                contours,
                hierarchy,
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
            val contoursPolyList: MutableList<MatOfPoint> = ArrayList<MatOfPoint>(contoursPoly.size)
            for (poly in contoursPoly) {
                contoursPolyList.add(MatOfPoint(*poly?.toArray()))
            }
            var highIndex = 0
            for (i in contours.indices) {
                val c: Scalar = scalarVals(color)
                Imgproc.drawContours(input, contoursPolyList, i, c)
                if (boundRect[i]!!.area() > varConfig.minRectArea) {
                    Imgproc.rectangle(input, boundRect[i]!!.tl(), boundRect[i]!!.br(), c, 2)
                }
                if (boundRect[i]!!.height > boundRect[highIndex]!!.height && boundRect[i]!!.width > boundRect[highIndex]!!.width) //get largest rectangle
                    highIndex = i
            }
            var lIndex = 0
            var rIndex = 0
            var tIndex = 0
            var bIndex = 0
            if (boundRect.size > 0) {
                // find furthest middle
                for (i in boundRect.indices) {
                    if (boundRect[i]!!.area() > varConfig.minRectArea) {
                        if (boundRect[i]!!.tl().x <= boundRect[lIndex]!!.tl().x) {
                            lIndex = i
                        }
                        if (boundRect[i]!!.br().x >= boundRect[rIndex]!!.br().x) {
                            rIndex = i
                        }
                        if (boundRect[i]!!.tl().y <= boundRect[tIndex]!!.tl().y) {
                            tIndex = i
                        }
                        if (boundRect[i]!!.br().y >= boundRect[bIndex]!!.br().y) {
                            bIndex = i
                        }
                    }
                }
            }
            if (boundRect.size > 0) {
                left = boundRect[lIndex]!!.tl().x
                right = boundRect[rIndex]!!.br().x
                top = boundRect[tIndex]!!.tl().y
                bottom = boundRect[bIndex]!!.br().y
                centerX = (left + (right - left) / 2).toInt()
                //Imgproc.putText(input, String.valueOf(centerX), new Point(middle + 7, top - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, scalarVals("white"), 1);
            }
            val height = Math.abs(top - bottom)
            val width = Math.abs(right - left)
            val centerX = (left + right) / 2
            val centerY = (top + bottom) / 2
            val newAspectRatio = width / height
            when (name) {
                "redcone" -> {
                    tolerance = redconeObjVars.tolerance
                    minWidth = redconeObjVars.minWidth
                    minHeight = redconeObjVars.minHeight
                    minArea = redconeObjVars.minArea
                    maxWidth = redconeObjVars.maxWidth
                    maxHeight = redconeObjVars.maxHeight
                    maxArea = redconeObjVars.maxArea
                }

                "bluecone" -> {
                    tolerance = blueconeObjVars.tolerance
                    minWidth = blueconeObjVars.minWidth
                    minHeight = blueconeObjVars.minHeight
                    minArea = blueconeObjVars.minArea
                    maxWidth = blueconeObjVars.maxWidth
                    maxHeight = blueconeObjVars.maxHeight
                    maxArea = blueconeObjVars.maxArea
                }

                "redprop" -> {
                    tolerance = redpropObjVars.tolerance
                    minWidth = redpropObjVars.minWidth
                    minHeight = redpropObjVars.minHeight
                    minArea = redpropObjVars.minArea
                    maxWidth = redpropObjVars.maxWidth
                    maxHeight = redpropObjVars.maxHeight
                    maxArea = redpropObjVars.maxArea
                }

                else -> {
                    tolerance = 0.1
                    minWidth = 0.0
                    minHeight = 0.0
                    minArea = 0.0
                    maxWidth = 0.0
                    maxHeight = 0.0
                    maxArea = 0.0
                }
            }
            val telemetry: Telemetry = FtcDashboard.getInstance().telemetry
            telemetry.addData("New Aspect Ratio", newAspectRatio)
            telemetry.addData("Aspect Ratio", aspectRatio)
            telemetry.addData("Width", width)
            telemetry.addData("Height", height)
            telemetry.addData("Center X", centerX)
            telemetry.addData("Center Y", centerY)
            // get recognitions translation
            if (minWidth <= width && minHeight <= height && maxWidth >= width && maxHeight >= height && minArea <= width * height && width * height <= maxArea) {
                if (aspectRatio + tolerance >= newAspectRatio && aspectRatio - tolerance <= newAspectRatio) {
                    //should be a cone
                    Imgproc.line(
                        input,
                        Point(centerX - 10, centerY),
                        Point(centerX + 10, centerY),
                        scalarVals("green")
                    )
                    Imgproc.line(
                        input,
                        Point(centerX, centerY - 10),
                        Point(centerX, centerY + 10),
                        scalarVals("green")
                    )
                    Imgproc.rectangle(
                        input,
                        Point(left, top),
                        Point(right, bottom),
                        scalarVals("green"),
                        2
                    )
                    val botDist: Double = Math.abs(input.height() - bottom)
                    val middle: Int = input.width() / 2
                    var xDist = Math.abs(middle - centerX)
                    if (centerX <= middle) xDist = -xDist
                    when (name) {
                        "redcone" -> {
                            xTranslation = xDist * redconeObjVars.translationX
                            yTranslation = botDist * redconeObjVars.translationY
                        }

                        "bluecone" -> {
                            xTranslation = xDist * blueconeObjVars.translationX
                            yTranslation = botDist * blueconeObjVars.translationY
                        }

                        "redprop" -> {
                            xTranslation = xDist * redpropObjVars.translationX
                            yTranslation = botDist * redpropObjVars.translationY
                        }

                        else -> {
                            xTranslation = 0.0
                            yTranslation = 0.0
                        }
                    }
                    Imgproc.line(
                        input,
                        Point(middle.toDouble(), 0.0),
                        Point(middle.toDouble(), input.height().toDouble()),
                        scalarVals("yellow"),
                        1
                    )
                    Imgproc.line(
                        input,
                        Point(0.0, (input.height() / 2).toDouble()),
                        Point(input.width().toDouble(), (input.height() / 2).toDouble()),
                        scalarVals("yellow"),
                        1
                    )
                }
            }
            Imgproc.line(
                input,
                Point((input.width() / 2).toDouble(), 0.0),
                Point((input.width() / 2).toDouble(), input.height().toDouble()),
                scalarVals("yellow"),
                1
            )
            Imgproc.line(
                input,
                Point(0.0, (input.height() / 2).toDouble()),
                Point(input.width().toDouble(), (input.height() / 2).toDouble()),
                scalarVals("yellow"),
                1
            )
            //            EOCVWebcam.centerX = centerX;
//            EOCVWebcam.centerY = centerY;
            telemetry.addData("x", centerX)
            telemetry.addData("y", centerY)
            telemetry.update()
            if (centerX <= 106) {
                Imgproc.putText(
                    input,
                    "1",
                    Point(left + 7, top - 10),
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    scalarVals(
                        color
                    ),
                    2
                )
            }
            if (centerX > 106 && centerX <= 112) {
                Imgproc.putText(
                    input,
                    "2",
                    Point(left + 7, top - 10),
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    scalarVals(
                        color
                    ),
                    2
                )
            }
            if (centerX > 112 && centerX <= 320) {
                Imgproc.putText(
                    input,
                    "3",
                    Point(left + 7, top - 10),
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    scalarVals(
                        color
                    ),
                    2
                )
            }
            return input
        }
    }

    class SamplePipeline : OpenCvPipeline() {
        override fun processFrame(input: Mat): Mat {
//            EOCVWebcam.pipelineName = "Sample Pipeline";
            val x: Int = input.width() / 2
            val y: Int = input.height() / 2
            //vert middle line
            Imgproc.line(
                input,
                Point(x.toDouble(), (y - 10).toDouble()),
                Point(x.toDouble(), (y + 10).toDouble()),
                scalarVals("red"),
                2
            )
            //horiz middle line
            Imgproc.line(
                input,
                Point((x - 10).toDouble(), y.toDouble()),
                Point((x + 10).toDouble(), y.toDouble()),
                scalarVals("red"),
                2
            )
            //Imgproc.line(input, new Point(input.rows(), input.cols()), new Point(0, 0), scalarVals("red"), 2);
            return input
        }
    }
}