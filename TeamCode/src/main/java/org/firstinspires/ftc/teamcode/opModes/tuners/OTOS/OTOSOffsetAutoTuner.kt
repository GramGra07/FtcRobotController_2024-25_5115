package org.firstinspires.ftc.teamcode.opModes.tuners.OTOS

import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles
import kotlin.math.abs
import kotlin.math.roundToInt

@TeleOp(group = GroupingTitles.TESTING)
@Disabled
//@Config
class OTOSOffsetAutoTuner : LinearOpMode() {
    private var maxHypotenuse = 0.0 // radius of the wrong circle
    private val timer = ElapsedTime()
    var data = mutableListOf<DataPoint>()
    private var xLimits = Pair(-9, 9)
    private var yLimits = Pair(-9, 9)

    lateinit var motorFrontLeft: DcMotor
    lateinit var motorBackLeft: DcMotor
    lateinit var motorFrontRight: DcMotor
    lateinit var motorBackRight: DcMotor
    lateinit var sparkFunOTOS: SparkFunOTOS

    override fun runOpMode() {

        //TODO: Set your sensor name and offsets
        sparkFunOTOS = hardwareMap.get(SparkFunOTOS::class.java, "spark")
        sparkFunOTOS.setLinearUnit(DistanceUnit.INCH)
        sparkFunOTOS.setAngularUnit(AngleUnit.DEGREES)
        sparkFunOTOS.linearScalar = 1.0
        sparkFunOTOS.angularScalar = 1.0
        sparkFunOTOS.calibrateImu()
        sparkFunOTOS.resetTracking()

        //TODO: Set your motor names
        motorFrontLeft = hardwareMap.get(DcMotor::class.java, "motorFrontLeft")
        motorBackLeft = hardwareMap.get(DcMotor::class.java, "motorBackLeft")
        motorFrontRight = hardwareMap.get(DcMotor::class.java, "motorFrontRight")
        motorBackRight = hardwareMap.get(DcMotor::class.java, "motorBackRight")

        //TODO: Set your motor directions
        motorBackLeft.direction = DcMotorSimple.Direction.REVERSE
        motorFrontLeft.direction = DcMotorSimple.Direction.REVERSE
        waitForStart()
        while (opModeIsActive()) {
            if (gamepad1.cross) {
                pause()
            }
            if (gamepad1.square) {
                play()
            }
            telemetry.addData("Press Square to start the test", "")
            telemetry.addData("You may choose to change the limits you test for", "")
            telemetry.addData("Press Cross to pause the test", "")
            telemetry.update()
        }
    }

    private fun getHyp(telemetry: Telemetry): Double {
        val hyp = sparkFunOTOS.getPose().toPoint()
            .distanceTo(Point(0.0, 0.0)) //recalculate the hypotenuse
        if (hyp > maxHypotenuse) {
            maxHypotenuse = hyp
        }
        telemetry.addData("hyp", hyp)
        telemetry.addData("max", maxHypotenuse)
        telemetry.update()
        return maxHypotenuse
    }

    private fun pause() {
        motorBackRight.power = 0.0
        motorBackLeft.power = 0.0
        motorFrontRight.power = 0.0
        motorFrontLeft.power = 0.0
    }

    fun power() {
        motorBackRight.power = -1.0
        motorBackLeft.power = 1.0
        motorFrontRight.power = -1.0
        motorFrontLeft.power = 1.0
    }

    fun play() {
        power()
        timer.reset()
        for (i in xLimits.first..xLimits.second) {
            for (j in yLimits.first..yLimits.second) {
                maxHypotenuse = 0.0
                if (breakout()) break
                sparkFunOTOS.position = Pose2D(0.0, 0.0, 0.0)
                sparkFunOTOS.offset = Pose2D(i.toDouble(), j.toDouble(), 0.0)
                while (sparkFunOTOS.position.h < 170) {
                    breakout()
                    telemetry.addData("x", sparkFunOTOS.position.x)
                    telemetry.addData("y", sparkFunOTOS.position.y)
                    telemetry.addData("h", sparkFunOTOS.position.h)
                    telemetry.addData("j", j)
                    telemetry.addData("i", i)
                    getHyp(telemetry)
                }

                val hyp = getHyp(telemetry)
                data.add(DataPoint(i.toDouble(), j.toDouble(), hyp))
            }
        }
        getMin(telemetry)
        val regression = runRegression(data, telemetry)
//        telemetry.addData("data",data)
//        telemetry.addData("newX", regression.first / 10)
//        telemetry.addData("newY", regression.second / 100)
        telemetry.update()
        while (!isStopRequested) {
            if (breakout()) break
            pause()
            telemetry.update()
        }
    }

    private fun breakout(): Boolean {
        return if (gamepad1.cross) {
            pause()
            true
        } else {
            false
        }
    }

    private fun getMin(telemetry: Telemetry): Pair<Double, Double> {
        var min = Double.POSITIVE_INFINITY
        var xOff = 0.0
        var yOff = 0.0
        for (point in data) {
            if (point.output < min) {
                min = point.output
                xOff = point.xOffset
                yOff = point.yOffset
            }
        }
        telemetry.addData("min", min)
        telemetry.addData("xOffSet", xOff)
        telemetry.addData("yOffset", yOff)
        return Pair(xOff, yOff)
    }

    private fun runRegression(data: List<DataPoint>, telemetry: Telemetry): Pair<Double, Double> {
        val (coefficients, intercept) = linearRegression(data)
        // Coarse search step size
        val coarseStep = 1.0
        var bestDifference = Double.POSITIVE_INFINITY
        var bestX = 0.0
        var bestY = 0.0

        // Coarse search: find an approximate optimal region
        for (x in xLimits.first..xLimits.second step coarseStep.toInt()) {
            for (y in yLimits.first..yLimits.second step coarseStep.toInt()) {
                val predictedOutput = predict(x.toDouble(), y.toDouble(), coefficients, intercept)
                val difference = abs(predictedOutput) // Absolute difference from 0

                if (difference < bestDifference) {
                    bestDifference = difference
                    bestX = x.toDouble()
                    bestY = y.toDouble()
                }
            }
        }

        // Fine search step size
        val fineStep = 0.01
        bestDifference = Double.POSITIVE_INFINITY // Reset the best difference

        // Fine search: refine the search around the best found in coarse search
        var xFine = bestX - coarseStep
        while (xFine <= bestX + coarseStep) {
            var yFine = bestY - coarseStep
            while (yFine <= bestY + coarseStep) {
                val predictedOutput = predict(xFine, yFine, coefficients, intercept)
                val difference = abs(predictedOutput)

                if (difference < bestDifference) {
                    bestDifference = difference
                    bestX = xFine
                    bestY = yFine
                }
                yFine += fineStep
            }
            xFine += fineStep
        }

        telemetry.addData("bestX", bestX)
        telemetry.addData("bestY", bestY)
        return Pair(bestX, bestY)
    }

    private fun predict(
        xOffset: Double,
        yOffset: Double,
        coefficients: DoubleArray,
        intercept: Double
    ): Double {
        return intercept + coefficients[0] * xOffset + coefficients[1] * yOffset
    }

    private fun linearRegression(data: List<DataPoint>): Pair<DoubleArray, Double> {
        val n = data.size

        // Calculate means
        val meanX = data.map { it.xOffset }.average()
        val meanY = data.map { it.yOffset }.average()
        val meanOutput = data.map { it.output }.average()

        // Calculate the coefficients for the regression equation: output = b0 + b1 * x + b2 * y
        var numeratorX = 0.0
        var numeratorY = 0.0
        var denominator = 0.0

        for (point in data) {
            val xDiff = point.xOffset - meanX
            val yDiff = point.yOffset - meanY
            val outputDiff = point.output - meanOutput

            numeratorX += xDiff * outputDiff
            numeratorY += yDiff * outputDiff
            denominator += xDiff * xDiff + yDiff * yDiff
        }

        val b1 = numeratorX / denominator
        val b2 = numeratorY / denominator
        val b0 = meanOutput - b1 * meanX - b2 * meanY

        return Pair(doubleArrayOf(b1, b2), b0)
    }

    fun SparkFunOTOS.getPose(): Pose2D {
        val newPose: Pose2D = Pose2D(this.position.x, this.position.y, -this.position.h)
        return newPose
    }

    private fun Point.distanceTo(point: Point): Double {
        val dx = point.x!! - this.x!!
        val dy = point.y!! - this.y!!
        return kotlin.math.sqrt(dx * dx + dy * dy)
    }

    private fun Pose2D.toPoint(): Point {
        return Point(this.x, this.y)
    }

    data class DataPoint(val xOffset: Double, val yOffset: Double, val output: Double)
    class Point(var x: Double? = 0.0, var y: Double? = 0.0) {
        fun setPoint(x: Double, y: Double) {
            this.x = x
            this.y = y
        }

        fun toPose(heading: Double? = 0.0): Pose2d {
            return Pose2d(x!!, y!!, Math.toRadians(heading!!))
        }

        override fun toString(): String {
            return "x: ${x?.roundToInt()}, y: ${y?.roundToInt()}"
        }
    }
}