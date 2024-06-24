package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.teamcode.extensions.MotorExtensions
import org.firstinspires.ftc.teamcode.ggutil.DriveConfig
import kotlin.math.cos
import kotlin.math.sin

class OdometrySubsystem3Wheel(
    ahwMap: HardwareMap,
    startX: Double,
    startY: Double,
    startHeading: Double
) {
    var poseX: Double
    var poseY: Double
    private var leftPod: DcMotorEx
    private var rightPod: DcMotorEx
    private var centerPod: DcMotorEx
    private var hwMap: HardwareMap

    private var trackwidth = DriveConfig.TRACK_WIDTH
    private var centerPodOffset = 18.7227783203125
    private var wheelRadius = DriveConfig.WHEEL_RADIUS
    private var podTicks = DriveConfig.TICKS_PER_REV
    private var cm_per_tick = 2.0 * Math.PI * wheelRadius / podTicks
    private var currentRightPod = 0
    private var currentLeftPod = 0
    private var currentCenterPod = 0
    private var oldRightPod = 0
    private var oldLeftPod = 0
    private var oldCenterPod = 0
    private var startHeading = 0.0
    private var botHeading = startHeading


    var heading = 0.0
    private var dtheta = 0.0
    private var dx = 0.0
    private var dy = 0.0
    private var imu: BNO055IMU
    private var yawAngle: Orientation? = null
    private var correctedStart = 0.0
    private var resetHeading = ElapsedTime()


    init {
        poseX = startX
        poseY = startY
        heading = startHeading
        this.startHeading = startHeading
        this.botHeading = startHeading
        resetHeading.reset()
        hwMap = ahwMap
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"
        parameters.loggingEnabled = true
        parameters.loggingTag = "IMU"
        imu = hwMap.get(BNO055IMU::class.java, "imu")
        imu.initialize(parameters)
        leftPod = MotorExtensions.initMotor(
            ahwMap,
            "LF",
            DcMotor.RunMode.RUN_WITHOUT_ENCODER,
            DcMotorSimple.Direction.REVERSE
        )
        leftPod.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        leftPod.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rightPod = MotorExtensions.initMotor(ahwMap, "RF", DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        rightPod.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightPod.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        centerPod = MotorExtensions.initMotor(
            ahwMap,
            "LB",
            DcMotor.RunMode.RUN_WITHOUT_ENCODER,
            DcMotorSimple.Direction.REVERSE
        )
        centerPod.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        centerPod.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

    }

    fun update() {
        yawAngle =
            imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)
        botHeading = (-yawAngle?.firstAngle!!).toDouble()
        botHeading += getCorrectStartHeading(startHeading)
        if (botHeading <= 0) {
            ConvertedHeadingForPosition = 360 + botHeading
        } else {
            ConvertedHeadingForPosition = 0 + botHeading
        }
        heading = ConvertedHeadingForPosition
        oldCenterPod = currentCenterPod
        oldLeftPod = currentLeftPod
        oldRightPod = currentRightPod
        currentCenterPod = -centerPod.currentPosition
        currentLeftPod = -leftPod.currentPosition
        currentRightPod = rightPod.currentPosition
        val dn1 = currentLeftPod - oldLeftPod
        val dn2 = currentRightPod - oldRightPod
        val dn3 = currentCenterPod - oldCenterPod
        dtheta = cm_per_tick * ((dn2 - dn1) / trackwidth)
        dx = cm_per_tick * (dn1 + dn2) / 2.0
        dy = cm_per_tick * (dn3 - (dn2 - dn1) * centerPodOffset / trackwidth)
        val theta = heading + dtheta / 2.0
        poseX += dx * cos(Math.toRadians(ConvertedHeadingForPosition)) - dy * sin(
            Math.toRadians(
                ConvertedHeadingForPosition
            )
        )
        poseY += dx * sin(Math.toRadians(ConvertedHeadingForPosition)) + dy * cos(
            Math.toRadians(
                ConvertedHeadingForPosition
            )
        )
        heading += dtheta
    }

    private fun getCorrectStartHeading(globalStartHeading: Double): Double {
        val startHeading = Math.round(globalStartHeading).toInt()
        when (startHeading) {
            270 -> {
                correctedStart = -90.0
            }

            180 -> {
                correctedStart = -180.0
            }

            90 -> {
                correctedStart = 90.0
            }
        }
        return correctedStart
    }

    fun reset(newPos: Vector2D) {
        leftPod.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightPod.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        centerPod.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        poseX = newPos.x
        poseY = newPos.y
        leftPod.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rightPod.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        centerPod.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun reset(heading: Double) {
        leftPod.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightPod.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        centerPod.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        this.heading = heading
        leftPod.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rightPod.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        centerPod.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun reset(newPos: Vector2D, heading: Double) {
        leftPod.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightPod.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        centerPod.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        poseX = newPos.x
        poseY = newPos.y
        this.heading = heading
        leftPod.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rightPod.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        centerPod.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    companion object {
        var ConvertedHeadingForPosition = 0.0
//        var poseX: Double = 0.0
//        var poseY: Double = 0.0
    }

    fun telemetry(telemetry: Telemetry) {
        telemetry.addData("X", poseX)
        telemetry.addData("Y", poseY)
        telemetry.addData("Heading", heading)
    }

    fun draw(packet: TelemetryPacket, rad: Double) {

        val poseX = poseX
        val poseY = poseY
        val heading = heading
        packet.fieldOverlay()
            .setFill("deepPink")
            .setStroke("deepPink")
            .setAlpha(1.0)
            .strokeCircle(poseX, poseY, rad)
            .strokeLine(
                poseX,
                poseY,
                poseX + rad / 2 * cos(Math.toRadians(heading)),
                poseY + rad / 2 * sin(Math.toRadians(heading))
            )
    }
}