package org.firstinspires.ftc.teamcode.opModes.testers.OTOS

import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig
import org.firstinspires.ftc.teamcode.extensions.OTOSExtension.getPose
import org.firstinspires.ftc.teamcode.extensions.PoseExtensions.distanceTo
import org.firstinspires.ftc.teamcode.extensions.PoseExtensions.toPoint
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles
import org.firstinspires.ftc.teamcode.utilClass.MathFunctions.Companion.inTolerance
import org.firstinspires.ftc.teamcode.utilClass.objects.Point
import kotlin.math.abs
import kotlin.math.sqrt

@TeleOp(group = GroupingTitles.TESTING)
//@Disabled
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
        val robot = HardwareConfig(this, false)

        //TODO: Set your sensor name and offsets
        sparkFunOTOS = hardwareMap.get(SparkFunOTOS::class.java, "spark")
        sparkFunOTOS.offset = SparkFunOTOS.Pose2D(0.0, 0.0, 0.0)
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

        waitForStart()
        while (opModeIsActive()) {
            if (gamepad1.cross) {
                pause()
            }
            if (gamepad1.square) {
                play()
            }
            telemetry.addData("Press Square to start the test","")
            telemetry.addData("You may choose to change the limits you test for","")
            telemetry.addData("Press Cross to pause the test","")
            telemetry.update()

            robot.drawPackets()
        }
    }
    private fun getHyp( telemetry: Telemetry):Double{
        val hyp = sparkFunOTOS.getPose().toPoint().distanceTo(Point(0.0, 0.0)) //recalculate the hypotenuse
        if (hyp > maxHypotenuse) {
            maxHypotenuse = hyp
        }
        telemetry.addData("hyp", hyp)
        telemetry.addData("max", maxHypotenuse)
        telemetry.update()
        return maxHypotenuse
    }
    private fun pause(){
        motorBackRight.power = 0.0
        motorBackLeft.power = 0.0
        motorFrontRight.power = 0.0
        motorFrontLeft.power = 0.0
    }
    fun play(){
        motorBackRight.power = 0.0
        motorBackLeft.power = 1.0
        motorFrontRight.power = 0.0
        motorFrontLeft.power = 1.0

        timer.reset()
        for (i in xLimits.first..xLimits.second){
            for (j in yLimits.first..yLimits.second){
                if (breakout()) break
                sparkFunOTOS.position = SparkFunOTOS.Pose2D(0.0, 0.0, 0.0)
                sparkFunOTOS.offset.x = i.toDouble()
                sparkFunOTOS.offset.y = j.toDouble()
                while(sparkFunOTOS.position.h < 355) {
                    if (breakout()) break
                    getHyp( telemetry)
                }
                val hyp = getHyp( telemetry)
                data.add(DataPoint(i.toDouble(), j.toDouble(), hyp))
            }
        }
        val regression = runRegression(data,telemetry)
        telemetry.addData("newX", regression.first)
        telemetry.addData("newY", regression.second)
        telemetry.update()
    }
    private fun breakout():Boolean{
        return if (gamepad1.cross){
            pause()
            true
        } else {
            false
        }
    }
    fun runRegression(data: List<DataPoint>, telemetry: Telemetry): Pair<Double, Double> {
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

        telemetry.addData("newX", bestX)
        telemetry.addData("newY", bestY)
        telemetry.update()
        return Pair(bestX, bestY)
    }
    private fun predict(xOffset: Double, yOffset: Double, coefficients: DoubleArray, intercept: Double): Double {
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
    data class DataPoint(val xOffset: Double, val yOffset: Double, val output:Double)
}