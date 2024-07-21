package org.firstinspires.ftc.teamcode.vision.openCV

import com.acmerobotics.dashboard.FtcDashboard
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.extensions.ScalarUtil.fetchScalar
import org.firstinspires.ftc.teamcode.extensions.ScalarUtil.scalarVals
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.varConfig
import org.firstinspires.ftc.teamcode.vision.openCV.trainer.vars.blueconeObjVars
import org.firstinspires.ftc.teamcode.vision.openCV.trainer.vars.redconeObjVars
import org.firstinspires.ftc.teamcode.vision.openCV.trainer.vars.redpropObjVars
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

class ColorDistance(var color: String) : OpenCvPipeline() {
        var end: Mat = Mat()
        var edges: Mat = Mat()
        var hierarchy: Mat = Mat()
        var ycrcbMat: Mat = Mat()
        var ycrcbMat2: Mat = Mat()
        var mask1: Mat = Mat()
        var mask2: Mat = Mat()
        override fun processFrame(input: Mat): Mat {
            val scalarLow: Scalar
            val scalarHigh: Scalar
            Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb)
            if (color != "red") {
                scalarLow = fetchScalar("l", color, 0)
                scalarHigh = fetchScalar("h", color, 0)
                Core.inRange(ycrcbMat, scalarLow, scalarHigh, end) 
            } else {
                ycrcbMat2 = ycrcbMat;
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
                Core.bitwise_or(mask1, mask2, end) 
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
                //Imgproc.drawContours(input, contoursPolyList, i, c)
                //Imgproc.rectangle(input, boundRect[i]!!.tl(), boundRect[i]!!.br(), c, 2)
                if (boundRect[i]!!.height > boundRect[highIndex]!!.height && boundRect[i]!!.width > boundRect[highIndex]!!.width) //get largest rectangle
                    highIndex = i
            }
            if (boundRect.size > 0) {
                val left = boundRect[highIndex]!!.tl().x
                val right = boundRect[highIndex]!!.br().x
                val top = boundRect[highIndex]!!.tl().y
                val bottom = boundRect[highIndex]!!.br().y
                val centerX = (left + (right - left) / 2).toInt()
                //Imgproc.putText(
                //    input,
                //     centerX.toString(),
                //    Point(left + 7, top - 10),
                    //Imgproc.FONT_HERSHEY_SIMPLEX,
                    //0.5,
                    //scalarVals(
                        //color
                    //),
                    //2
                //)
                Imgproc.rectangle(input, boundRect[i]!!.tl(), boundRect[i]!!.br(), c, 2)
            }
            
            return input
        }
    }
    }
    data class distanceRecognizerDetails(val height,
    val width,
    val inToPixels,//pix/in
    j
    ){
    val pixToIn = 1/inToPixels
    fun convertPixToIn(var pix:double):double{
    return pix*pixToIn
    }
    fun convertInToPix(var inches:double):double{
    return inToPixels*inches}
    }