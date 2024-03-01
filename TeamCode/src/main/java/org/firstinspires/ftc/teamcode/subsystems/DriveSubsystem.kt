package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.UtilClass.FileWriterFTC
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.varConfig
import org.firstinspires.ftc.teamcode.extensions.MotorExtensions
import org.firstinspires.ftc.teamcode.extensions.MotorExtensions.getMotorCurrent
import org.firstinspires.ftc.teamcode.opModes.DistanceStorage
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage
import kotlin.math.sqrt

class DriveSubsystem(ahwMap: HardwareMap) {
    var drive: MecanumDrive? = null

    private var motorFrontLeft: DcMotorEx? = null
    private var motorBackLeft: DcMotorEx? = null
    private var motorFrontRight: DcMotorEx? = null
    private var motorBackRight: DcMotorEx? = null
    private var motorList: List<DcMotorEx> = listOf()

    init {
        if (drive == null) {
            drive = MecanumDrive(ahwMap)
            drive!!.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            drive!!.poseEstimate = PoseStorage.currentPose
        }
        if (motorFrontLeft == null) {
            motorFrontLeft =
                MotorExtensions.initMotor(
                    ahwMap,
                    "motorFrontLeft",
                    DcMotor.RunMode.RUN_WITHOUT_ENCODER
                )
            motorList.plus(motorFrontLeft!!)
        }
        if (motorBackLeft == null) {
            motorBackLeft = MotorExtensions.initMotor(
                ahwMap,
                "motorBackLeft",
                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                DcMotorSimple.Direction.REVERSE
            )
            motorList.plus(motorBackLeft!!)
        }
        if (motorFrontRight == null) {
            motorFrontRight =
                MotorExtensions.initMotor(
                    ahwMap,
                    "motorFrontRight",
                    DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                    DcMotorSimple.Direction.REVERSE
                )
            motorList.plus(motorFrontRight!!)
        }
        if (motorBackRight == null) {
            motorBackRight =
                MotorExtensions.initMotor(
                    ahwMap,
                    "motorBackRight",
                    DcMotor.RunMode.RUN_WITHOUT_ENCODER
                )
            motorList.plus(motorBackRight!!)
        }
    }

    private var thisDist = 0.0
    private var slowMult: Int = varConfig.slowMult
    private var slowPower = 0
    private var xControl = 0.0
    private var yControl = 0.0
    private var frontRightPower = 0.0
    private var frontLeftPower = 0.0
    private var backRightPower = 0.0
    private var backLeftPower = 0.0
    private var gamepadX = 0.0
    private var gamepadY = 0.0
    private var gamepadHypot = 0.0
    private var controllerAngle = 0.0
    private var robotDegree = 0.0
    private var movementDegree = 0.0
    var slowModeIsOn = false
    var reverse = false
    var isAutoInTeleop = false
    fun driveByGamepads(fieldCentric: Boolean, myOpMode: OpMode) {
        if (fieldCentric) {
            gamepadX =
                myOpMode.gamepad1.left_stick_x.toDouble() //get the x val of left stick and store
            gamepadY =
                -myOpMode.gamepad1.left_stick_y.toDouble() //get the y val of left stick and store
            gamepadHypot = Range.clip(Math.hypot(gamepadX, gamepadY), 0.0, 1.0) //get the
            // hypotenuse of the x and y values,clip it to a max of 1 and store
            controllerAngle = Math.toDegrees(
                Math.atan2(
                    gamepadY,
                    gamepadX
                )
            ) //Get the angle of the controller stick using arc tangent
            robotDegree = Math.toDegrees(drive!!.poseEstimate.heading) // change to imu
            movementDegree =
                controllerAngle - robotDegree //get the movement degree based on the controller vs robot angle
            xControl =
                Math.cos(Math.toRadians(movementDegree)) * gamepadHypot //get the x value of the movement
            yControl =
                Math.sin(Math.toRadians(movementDegree)) * gamepadHypot //get the y value of the movement
            val turn: Double = -myOpMode.gamepad1.right_stick_x.toDouble()
            frontRightPower =
                (yControl * Math.abs(yControl) - xControl * Math.abs(xControl) + turn) / slowPower
            backRightPower =
                (yControl * Math.abs(yControl) + xControl * Math.abs(xControl) + turn) / slowPower
            frontLeftPower =
                (yControl * Math.abs(yControl) + xControl * Math.abs(xControl) - turn) / slowPower
            backLeftPower =
                (yControl * Math.abs(yControl) - xControl * Math.abs(xControl) - turn) / slowPower
        } else {
//            reverse = myOpMode.gamepad1.touchpad_finger_1_x > 0.5;//0,1 left to right
//            reversed = reverse;
            yControl = -myOpMode.gamepad1.left_stick_y.toDouble()
            xControl = myOpMode.gamepad1.left_stick_x.toDouble()
            if (reverse) {
                yControl = -yControl
                xControl = -xControl
            }
            val turn: Double = -myOpMode.gamepad1.right_stick_x.toDouble()
            slowPower = if (slowModeIsOn) {
                slowMult
            } else {
                1
            }
            frontRightPower = (yControl - xControl + turn) / slowPower
            backRightPower = (yControl + xControl + turn) / slowPower
            frontLeftPower = (yControl + xControl - turn) / slowPower
            backLeftPower = (yControl - xControl - turn) / slowPower
        }
        drive!!.update()
        updateDistTraveled(PoseStorage.currentPose, drive!!.poseEstimate)
        FileWriterFTC.writeToFile(
            HardwareConfig.fileWriter!!,
            drive!!.poseEstimate.x.toInt(),
            drive!!.poseEstimate.y.toInt()
        )
        PoseStorage.currentPose = drive!!.poseEstimate
    }

    private fun updateDistTraveled(before: Pose2d, after: Pose2d) {
        val x = after.x - before.x
        val y = after.y - before.y
        val dist = sqrt(x * x + y * y)
        thisDist += dist
        DistanceStorage.totalDist += dist
    }

    fun stop() {
        motorFrontLeft!!.power = 0.0
        motorBackLeft!!.power = 0.0
        motorFrontRight!!.power = 0.0
        motorBackRight!!.power = 0.0
    }

    fun reset() {
        thisDist = 0.0
    }

    private fun power() {
        if (isAutoInTeleop) { //! verify this works
            motorFrontLeft!!.power = frontLeftPower
            motorBackLeft!!.power = backLeftPower
            motorFrontRight!!.power = frontRightPower
            motorBackRight!!.power = backRightPower
        }
    }

    fun update() {
        power()
        drive!!.update()
    }

    fun telemetry(telemetry: Telemetry) {
        if (reverse) {
            telemetry.addData("reversed", "")
        }
        if (slowModeIsOn) {
            telemetry.addData("slowMode", "")
        }
        telemetry.addData("thisDistance (in)", "%.1f", thisDist)
        telemetry.addData("totalDistance (in)", "%.1f", DistanceStorage.totalDist)
        getCurrentTelemetry(telemetry)
    }

    private fun getCurrentTelemetry(telemetry: Telemetry) {
        val currentList: Map<DcMotorEx, Double> = mapOf()
        motorList.forEach {
            currentList.plus(Pair(it, it.getMotorCurrent()))
        }
        currentList.forEach {
            telemetry.addData(it.key.deviceName, it.value)
        }
    }
}