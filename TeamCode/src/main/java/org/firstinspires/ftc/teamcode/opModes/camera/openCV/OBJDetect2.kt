package org.firstinspires.ftc.teamcode.opModes.camera.openCV

import org.firstinspires.ftc.teamcode.Enums.Alliance
import org.firstinspires.ftc.teamcode.Enums.AutoRandom
import org.firstinspires.ftc.teamcode.extensions.Extensions.ledIND
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.green1
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.green2
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.green3
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.green4
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline

//@Config
class OBJDetect2(var alliance: Alliance) : OpenCvPipeline() {
    var ycrcbMat = Mat()
    var right = Mat()
    var middle = Mat()
    var c = Scalar(255.0, 0.0, 0.0)
    var current = 0
    var scalarLow: Scalar? = null
    var scalarHigh: Scalar? = null
    override fun processFrame(input: Mat): Mat {
        if (alliance == Alliance.RED) {
            scalarLow = Scalar(0.0, 147.0, 0.0)
            scalarHigh = Scalar(255.0, 255.0, 255.0)
        } else if (alliance == Alliance.BLUE) {
            scalarLow = Scalar(0.0, 0.0, 130.0)
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
        Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb)
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
        if (rightMean[0] > scalarLow!!.`val`[0] && rightMean[0] < scalarHigh!!.`val`[0]) {
            if (rightMean[1] > scalarLow!!.`val`[1] && rightMean[1] < scalarHigh!!.`val`[1]) {
                if (rightMean[2] > scalarLow!!.`val`[2] && rightMean[2] < scalarHigh!!.`val`[2]) {
                    Imgproc.rectangle(
                        input, Point(pointsX[2].toDouble(), pointsY[2].toDouble()), Point(
                            pointsX[3].toDouble(), pointsY[3].toDouble()
                        ), Scalar(0.0, 255.0, 0.0), 1
                    )
                    Imgproc.putText(
                        input,
                        "right",
                        Point((input.width() / 2).toDouble(), (input.height() / 2).toDouble()),
                        0,
                        5.0,
                        Scalar(0.0, 255.0, 0.0)
                    )
                    autoHardware.autonomousRandom = AutoRandom.right
                    green1.ledIND(HardwareConfig.red1, false)
                    green2.ledIND(HardwareConfig.red2, false)
                    green3.ledIND(HardwareConfig.red3, true)
                    green4.ledIND(HardwareConfig.red3, false)
                    current = 1
                }
            }
        }
        if (current != 1) {
            if (middleMean[0] > scalarLow!!.`val`[0] && middleMean[0] < scalarHigh!!.`val`[0]) {
                if (middleMean[1] > scalarLow!!.`val`[1] && middleMean[1] < scalarHigh!!.`val`[1]) {
                    if (middleMean[2] > scalarLow!!.`val`[2] && middleMean[2] < scalarHigh!!.`val`[2]) {
                        Imgproc.rectangle(
                            input, Point(pointsX[0].toDouble(), pointsY[0].toDouble()), Point(
                                pointsX[1].toDouble(), pointsY[1].toDouble()
                            ), Scalar(0.0, 255.0, 0.0), 1
                        )
                        Imgproc.putText(
                            input,
                            "middle",
                            Point((input.width() / 2).toDouble(), (input.height() / 2).toDouble()),
                            0,
                            5.0,
                            Scalar(0.0, 255.0, 0.0)
                        )
                        autoHardware.autonomousRandom = AutoRandom.mid
                        green1.ledIND(HardwareConfig.red1, true)
                        green2.ledIND(HardwareConfig.red2, true)
                        green3.ledIND(HardwareConfig.red3, false)
                        green4.ledIND(HardwareConfig.red4, true)
                        current = 1
                    }
                }
            }
        }
        if (current == 0) {
            Imgproc.putText(
                input,
                "left",
                Point((input.width() / 2).toDouble(), (input.height() / 2).toDouble()),
                0,
                5.0,
                Scalar(0.0, 255.0, 0.0)
            )
            autoHardware.autonomousRandom = AutoRandom.left
            green1.ledIND(HardwareConfig.red1, false)
            green2.ledIND(HardwareConfig.red2, true)
            green3.ledIND(HardwareConfig.red3, false)
            green4.ledIND(HardwareConfig.red4, false)
        }
        current = 0
        ycrcbMat.release()
        right.release()
        middle.release()
        return input
    }

    companion object {
        var pointsX = intArrayOf(560, 560 + 120, 100, 250)
        var pointsY = intArrayOf(70, 200, 40, 190)
    }
}