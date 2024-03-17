package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
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
import org.firstinspires.ftc.teamcode.extensions.PoseExtensions.toPoint
import org.firstinspires.ftc.teamcode.opModes.DistanceStorage
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage
import org.firstinspires.ftc.teamcode.subsystems.avoidance.AvoidanceSubsystem
import org.firstinspires.ftc.teamcode.subsystems.avoidance.VectorField.Companion.getCorrectionByAvoidance
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.math.sqrt

@Config
class DriveSubsystem(ahwMap: HardwareMap) {
    @JvmField
    var deadZone = 0.15

    var drive: MecanumDrive? = null

    private var motorFrontLeft: DcMotorEx? = null
    private var motorBackLeft: DcMotorEx? = null
    private var motorFrontRight: DcMotorEx? = null
    private var motorBackRight: DcMotorEx? = null
    private var motorList: List<DcMotorEx> = listOf()
    private var odometrySubsystem: OdometrySubsystem2Wheel? = null
    private var avoidanceSubsystem: AvoidanceSubsystem? = null

    init {
        if (odometrySubsystem == null) {
            odometrySubsystem = OdometrySubsystem2Wheel(ahwMap, 0.0, 0.0, 0.0)
        }
        if (avoidanceSubsystem == null) {
            avoidanceSubsystem = AvoidanceSubsystem()
        }
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
            )
            motorBackLeft!!.direction = DcMotorSimple.Direction.REVERSE
            motorList.plus(motorBackLeft!!)
        }
        if (motorFrontRight == null) {
            motorFrontRight =
                MotorExtensions.initMotor(
                    ahwMap,
                    "motorFrontRight",
                    DcMotor.RunMode.RUN_WITHOUT_ENCODER
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
    private var slowPower = 1
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
        slowPower = if (slowModeIsOn) {
            slowMult
        } else {
            1
        }
        if (fieldCentric) {
            gamepadX =
                myOpMode.gamepad1.left_stick_x.toDouble() //get the x val of left stick and store
            gamepadY =
                -myOpMode.gamepad1.left_stick_y.toDouble() //get the y val of left stick and store
            gamepadHypot = Range.clip(hypot(gamepadX, gamepadY), 0.0, 1.0) //get the
            // hypotenuse of the x and y values,clip it to a max of 1 and store
            controllerAngle = Math.toDegrees(
                atan2(
                    gamepadY,
                    gamepadX
                )
            ) //Get the angle of the controller stick using arc tangent
            robotDegree = Math.toDegrees(drive!!.poseEstimate.heading) // change to imu
            movementDegree =
                controllerAngle - robotDegree //get the movement degree based on the controller vs robot angle
            xControl =
                cos(Math.toRadians(movementDegree)) * gamepadHypot //get the x value of the movement
            yControl =
                sin(Math.toRadians(movementDegree)) * gamepadHypot //get the y value of the movement
            val turn: Double = (-myOpMode.gamepad1.right_stick_x).toDouble()
            frontRightPower =
                (yControl * abs(yControl) - xControl * abs(xControl) + turn) / slowPower
            backRightPower =
                (yControl * abs(yControl) + xControl * abs(xControl) + turn) / slowPower
            frontLeftPower =
                (yControl * abs(yControl) + xControl * abs(xControl) - turn) / slowPower
            backLeftPower =
                (yControl * abs(yControl) - xControl * abs(xControl) - turn) / slowPower
        } else {

            gamepadX =
                myOpMode.gamepad1.left_stick_x.toDouble() //get the x val of left stick and store

            gamepadY =
                -myOpMode.gamepad1.left_stick_y.toDouble() //get the y val of left stick and store

            gamepadHypot = Range.clip(hypot(gamepadX, gamepadY), 0.0, 1.0) //get the

            // hypotenuse of the x and y values,clip it to a max of 1 and store
            // hypotenuse of the x and y values,clip it to a max of 1 and store
            controllerAngle = Math.toDegrees(
                atan2(
                    gamepadY,
                    gamepadX
                )
            ) //Get the angle of the controller stick using arc tangent

            robotDegree = Math.toDegrees(drive!!.poseEstimate.heading) // change to imu

            movementDegree =
                controllerAngle - robotDegree //get the movement degree based on the controller vs robot angle

            xControl =
                cos(Math.toRadians(movementDegree)) * gamepadHypot //get the x value of the movement

            yControl =
                sin(Math.toRadians(movementDegree)) * gamepadHypot //get the y value of the movement

            val turn = -myOpMode.gamepad1.right_stick_x.toDouble()
            frontRightPower =
                (yControl * abs(yControl) - xControl * abs(xControl) + turn) / slowPower
            backRightPower =
                (yControl * abs(yControl) + xControl * abs(xControl) + turn) / slowPower
            frontLeftPower =
                (yControl * abs(yControl) + xControl * abs(xControl) - turn) / slowPower
            backLeftPower = (yControl * abs(yControl) - xControl * abs(xControl) - turn) / slowPower
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

    fun resetHeading() {
        drive!!.poseEstimate = Pose2d(drive!!.poseEstimate.x, drive!!.poseEstimate.y, 0.0)
    }

    private fun power() {
        avoidanceSubsystem!!.update()
        if (!isAutoInTeleop) {
            val addedPowers: Map<String, Double?> = disectPowers()
            val flP: Double = addedPowers["FL"] ?: 0.0
            val frP: Double = addedPowers["FR"] ?: 0.0
            val rlP: Double = addedPowers["RL"] ?: 0.0
            val rrP: Double = addedPowers["RR"] ?: 0.0
            frontLeftPower = Range.clip(frontLeftPower + flP, -1.0, 1.0)
            frontRightPower = Range.clip(frontRightPower + frP, -1.0, 1.0)
            backLeftPower = Range.clip(backLeftPower + rlP, -1.0, 1.0)
            backRightPower = Range.clip(backRightPower + rrP, -1.0, 1.0)

            motorFrontLeft!!.power = frontLeftPower
            motorBackLeft!!.power = backLeftPower
            motorFrontRight!!.power = frontRightPower
            motorBackRight!!.power = backRightPower
        }
    }

    private fun disectPowers(): Map<String, Double> {
        return getCorrectionByAvoidance(avoidanceSubsystem!!.fields, drive!!.poseEstimate.toPoint())
    }

    fun update() {
        power()
        drive!!.update()
        odometrySubsystem!!.update()
        drawBot(
            TelemetryPacket(),
            FtcDashboard.getInstance(),
            drive!!.poseEstimate.x,
            drive!!.poseEstimate.y,
            drive!!.poseEstimate.heading
        )
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
        odometrySubsystem!!.telemetry(telemetry)
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

    private fun drawBot(
        packet: TelemetryPacket,
        dashboard: FtcDashboard,
        poseX: Double,
        poseY: Double,
        heading: Double
    ) {
        packet.fieldOverlay()
            .setFill("pink")
            .setAlpha(1.0)
            .fillCircle(poseX, poseY, 2.0)
            .strokeLine(
                poseX,
                poseY,
                poseX + 10 * cos(Math.toRadians(heading)),
                poseY + 10 * sin(Math.toRadians(heading))
            )

        dashboard.sendTelemetryPacket(packet)
    }
}