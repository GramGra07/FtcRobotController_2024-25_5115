package org.firstinspires.ftc.teamcode.ggutil.openCV.trainer

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig.Companion.CAM1
import org.firstinspires.ftc.teamcode.extensions.ScalarUtil.fetchScalar
import org.firstinspires.ftc.teamcode.extensions.ScalarUtil.scalarVals
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Point
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline
import org.openftc.easyopencv.OpenCvWebcam

@TeleOp
@Disabled //@Config

class ObjectRecognitionTrainer : LinearOpMode() {
    lateinit var webcam: OpenCvWebcam
    var aspectRatio = 0.0
    var minWidth = 0.0
    var minHeight = 0.0
    var maxWidth = 0.0
    var maxHeight = 0.0
    var translationX = 0.0
    var translationY = 0.0
    var heightError = ""
    var widthError = ""
    var areaError = ""
    var aspectError1 = ""
    var aspectError2 = ""
    var xError = ""
    var yError = ""
    var centerX = 0.0
    var left = 0.0
    var right = 0.0
    var top = 0.0
    var bottom = 0.0
    var centerY = 0.0
    var height = 0.0
    var width = 0.0
    var xDist = 0.0
    var botDist = 0.0
    var middle = 0.0

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val telemetry: Telemetry =
            MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().telemetry)
        val pipeline: OpenCvPipeline = ColorIsolateBound()
        val cameraMonitorViewId: Int = hardwareMap.appContext.resources
            .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap.get<WebcamName>(
                WebcamName::class.java, CAM1
            ), cameraMonitorViewId
        )
        webcam.setPipeline(pipeline)
        FtcDashboard.getInstance().startCameraStream(webcam, 0.0)
        webcam.setMillisecondsPermissionTimeout(5000) // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
            }

            override fun onError(errorCode: Int) {}
        })
        telemetry.addLine("Please navigate to FTC Dashboard @ http://192.168.43.1:8080/dash")
        telemetry.addLine("Also have the camera and configuration tab open on the Dashboard")
        telemetry.addLine("Please open the ObjectRecognitionTrainer in the configuration tab, this is where you will edit the variables")
        telemetry.addLine("Press enter to save any changes made on FTC Dash")
        telemetry.addLine("Please enter the name and color of the object you are training for in FTC Dash")
        telemetry.addLine("If you just now entered name and color, please hit stop and re-start this program")
        telemetry.addLine("Note that while training, make sure that it only sees the object you want it to")
        telemetry.addLine("Start the OpMode")
        telemetry.update()
        waitForStart()
        telemetry.clearAll()
        if (opModeIsActive()) {
            if (color === "" || name === "") {
                telemetry.addData(
                    "Please enter the name and color of the object you are training for in FTC Dash",
                    "stop the opmode and restart"
                )
                telemetry.update()
                while (!gamepad1.square) {
                    if (isStopRequested) {
                        return
                    }
                }
            }
            telemetry.clearAll()
            telemetry.addData(
                "Please move the object you are training for into the frame",
                "press circle on gamepad to capture"
            )
            telemetry.update()
            while (!gamepad1.circle) {
                if (isStopRequested) {
                    return
                }
            }
            aspectRatio = width / height
            telemetry.clearAll()
            telemetry.addData(
                "Now place the object as far back as you want it to read",
                "press square on gamepad to capture"
            )
            telemetry.update()
            while (!gamepad1.square) {
                if (isStopRequested) {
                    return
                }
            }
            minWidth = width
            minHeight = height
            telemetry.clearAll()
            telemetry.addData(
                "Now place the object as close as you want it to read",
                "press triangle on gamepad to capture"
            )
            telemetry.update()
            while (!gamepad1.triangle) {
                if (isStopRequested) {
                    return
                }
            }
            maxWidth = width
            maxHeight = height
            telemetry.clearAll()
            telemetry.addData(
                "Please move object back to starting position",
                "press square on gamepad when complete"
            )
            telemetry.update()
            while (!gamepad1.square) {
                if (isStopRequested) {
                    return
                }
            }
            telemetry.clearAll()
            // translations
            telemetry.addData(
                "Please measure the distance from the center of the camera to the center of the object and input it in FTC Dash under xDistance",
                ""
            )
            telemetry.addData(
                "Please measure the distance from the camera to the front of the object and input it in FTC Dash under yDistance",
                ""
            )
            telemetry.addLine("Press circle on gamepad to complete")
            telemetry.update()
            while (!gamepad1.circle) {
                if (isStopRequested) {
                    return
                }
            }
            if (botDist == 0.0) {
                telemetry.addData(
                    "Object too close, please move robot back",
                    "press triangle on gamepad when done"
                )
                telemetry.update()
                while (!gamepad1.triangle) {
                    if (isStopRequested) {
                        return
                    }
                }
            }
            telemetry.clearAll()
            webcam.stopStreaming()
            translationX = Math.abs(xDistance / xDist)
            translationY = yDistance / botDist
            val fullName = color + name
            val caseName = "\"" + fullName + "\""
            telemetry.addLine("Building file, you will create a new java file with the name: " + fullName + "ObjVars.java")
            telemetry.update()
            sleep(1500)
            telemetry.addLine("Now running check on variables created")
            telemetry.update()
            sleep(1500)
            checkVars()
            buildFile()
            telemetry.addData(
                "File built, please copy and paste the file into your project",
                "press square on gamepad when done"
            )
            telemetry.update()
            while (!gamepad1.square) {
                if (isStopRequested) {
                    return
                }
            }
            telemetry.clearAll()
            telemetry.addLine("To finish setup you will add the following to the Object Recognition Pipeline")
            telemetry.addLine("In the constructor add the following lines: ")
            telemetry.addLine("case $caseName:")
            telemetry.addLine("aspectRatio = " + fullName + "ObjVars.aspectRatio;")
            telemetry.addLine("break;")
            telemetry.addLine("Press circle on gamepad to continue")
            telemetry.update()
            while (!gamepad1.circle) {
                if (isStopRequested) {
                    return
                }
            }
            telemetry.clearAll()
            telemetry.addLine("Add the following lines under process frame and the switch statement")
            telemetry.addLine("case $caseName:")
            telemetry.addLine("tolerance = " + fullName + "ObjVars.tolerance;")
            telemetry.addLine("minWidth = " + fullName + "ObjVars.minWidth;")
            telemetry.addLine("minHeight = " + fullName + "ObjVars.minHeight;")
            telemetry.addLine("minArea = " + fullName + "ObjVars.minArea;")
            telemetry.addLine("maxWidth = " + fullName + "ObjVars.maxWidth;")
            telemetry.addLine("maxHeight = " + fullName + "ObjVars.maxHeight;")
            telemetry.addLine("maxArea = " + fullName + "ObjVars.maxArea;")
            telemetry.addLine("break;")
            telemetry.addLine("Press square on gamepad to continue")
            telemetry.update()
            while (!gamepad1.square) {
                if (isStopRequested) {
                    return
                }
            }
            telemetry.clearAll()
            telemetry.addLine("Add the following lines under get recognitions (in a comment) and the switch statement")
            telemetry.addLine("case $caseName:")
            telemetry.addLine("xTranslation = xDist * " + fullName + "ObjVars.translationX;")
            telemetry.addLine("yTranslation = botDist * " + fullName + "ObjVars.translationY;")
            telemetry.addLine("break;")
            telemetry.addData("Once done", "press triangle to continue")
            telemetry.update()
            while (!gamepad1.triangle) {
                if (isStopRequested) {
                    return
                }
            }
            telemetry.clearAll()
            telemetry.addLine("All finished, now just change the name in pieplineTester of whatever opmode to the color and name and it should work!")
            telemetry.addLine("Please press stop when done")
            telemetry.update()
            webcam.closeCameraDevice()
        }
    }

    fun checkVars() {
        val telemetry: Telemetry =
            MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().telemetry)
        telemetry.addLine("Possible errors:")
        if (minHeight > maxHeight) heightError = "// possible height error minHeight>maxHeight"
        if (minWidth > maxWidth) widthError = "// possible width error minWidth>maxWidth"
        if (minWidth * minHeight > maxWidth * maxHeight) areaError =
            "// possible area error minArea>maxArea"
        if (minHeight < 0) heightError = "// possible height error minHeight<0"
        if (minWidth < 0) widthError = "// possible width error minWidth<0"
        if (minWidth * minHeight < 0) areaError = "// possible area error minArea<0"
        val maxAspect = maxWidth / maxHeight
        val minAspect = minWidth / minHeight
        if (!(aspectRatio + tolerance >= maxAspect) && !(maxAspect >= aspectRatio - tolerance)) aspectError2 =
            "// possible aspect ratio error, max's not in aspect"
        if (!(aspectRatio + tolerance >= minAspect) && !(minAspect >= aspectRatio - tolerance)) aspectError1 =
            "// possible aspect ratio error, min's not in aspect"
        if (translationX > 100) xError = "// possible translation error, xTranslation>100"
        if (translationY > 100) yError = "// possible translation error, yTranslation>100"
    }

    fun buildFile() {
        val telemetry: Telemetry =
            MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().telemetry)
        telemetry.clearAll()
        telemetry.addData("Name", color + name + "ObjVars.java")
        telemetry.addLine("package ; // set this to your correct package")
        telemetry.addLine("import com.acmerobotics.dashboard.config.Config;")
        telemetry.addLine("@Config")
        telemetry.addLine("public class " + color + name + "ObjVars {")
        telemetry.addLine("public static double aspectRatio = $aspectRatio;$aspectError1$aspectError2")
        telemetry.addLine("public static double minWidth = $minWidth;$widthError")
        telemetry.addLine("public static double minHeight = $minHeight;$heightError")
        telemetry.addLine("public static double maxWidth = $maxWidth;")
        telemetry.addLine("public static double maxHeight = $maxHeight;")
        telemetry.addLine("public static double minArea = minWidth * minHeight;$areaError")
        telemetry.addLine("public static double maxArea = maxWidth * maxHeight;")
        telemetry.addLine("public static double tolerance = " + tolerance + "; // this is the value that will determine how far off the aspect ratio can be to still detect it, you will need to tune it more")
        telemetry.addLine("public static double translationX = $translationX;$xError")
        telemetry.addLine("public static double translationY = $translationY;$yError")
        telemetry.addLine("}")
        telemetry.addLine("Copy this code into the new file you created and set the package name")
        telemetry.update()
    }

    inner class ColorIsolateBound : OpenCvPipeline() {
        var color = Companion.color
        var hsv: Mat = Mat()
        var hsv2: Mat = Mat()
        var mask1: Mat = Mat()
        var mask2: Mat = Mat()
        var end: Mat = Mat()
        var edges: Mat = Mat()
        var hierarchy: Mat = Mat()
        override fun processFrame(input: Mat): Mat {
            lateinit var scalarLow: Scalar
            lateinit var scalarHigh: Scalar
            lateinit var place: Scalar
            lateinit var place2: Scalar
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV) //change to hsv
            Imgproc.cvtColor(input, hsv2, Imgproc.COLOR_RGB2HSV)
            if (color != "red") {
                scalarLow = fetchScalar("l", color, 0)
                scalarHigh = fetchScalar("h", color, 0)
                Core.inRange(hsv, scalarLow, scalarHigh, end) //detect color, output to end
            } else {
                Core.inRange(hsv, fetchScalar("l", color, 1), fetchScalar("h", color, 1), mask1)
                Core.inRange(hsv2, fetchScalar("l", color, 2), fetchScalar("h", color, 2), mask2)
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
                lateinit var c: Scalar
                c = scalarVals(color)
                Imgproc.drawContours(input, contoursPolyList, i, c)
                Imgproc.rectangle(input, boundRect[i]!!.tl(), boundRect[i]!!.br(), c, 2)
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
            if (boundRect.size > 0) {
                left = boundRect[lIndex]!!.tl().x
                right = boundRect[rIndex]!!.br().x
                top = boundRect[tIndex]!!.tl().y
                bottom = boundRect[bIndex]!!.br().y
                centerX = (left + (right - left) / 2).toInt().toDouble()
                //Imgproc.putText(input, String.valueOf(centerX), new Point(middle + 7, top - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, scalarVals("white"), 1);
            }
            middle = (input.width() / 2).toDouble()
            Imgproc.line(
                input,
                Point(middle, 0.0),
                Point(middle, input.height().toDouble()),
                scalarVals("yellow"),
                1
            )
            height = Math.abs(top - bottom)
            width = Math.abs(right - left)
            centerX = (left + right) / 2
            centerY = (top + bottom) / 2
            aspectRatio = width / height
            botDist = Math.abs(input.height() - bottom)
            xDist = Math.abs(middle - centerX)
            if (centerX <= middle) xDist = -xDist
            Imgproc.line(input, Point(centerX, top), Point(centerX, 0.0), scalarVals(color), 1)
            Imgproc.line(
                input,
                Point(centerX, bottom),
                Point(centerX, input.height().toDouble()),
                scalarVals(color),
                1
            )
            Imgproc.putText(
                input,
                Math.abs(xDist).toString(),
                Point((centerX + middle) / 2, 15.0),
                Imgproc.FONT_HERSHEY_SIMPLEX,
                0.5,
                scalarVals("white"),
                1
            )
            Imgproc.putText(
                input,
                botDist.toString(),
                Point(centerX + 2, (bottom + input.height()) / 2),
                Imgproc.FONT_HERSHEY_SIMPLEX,
                0.5,
                scalarVals("white"),
                1
            )
            return input
        }
    }

    companion object {
        var name = ""
        var color = ""
        var tolerance = 0.2
        var xDistance = 0.0
        var yDistance = 0.0
    }
}