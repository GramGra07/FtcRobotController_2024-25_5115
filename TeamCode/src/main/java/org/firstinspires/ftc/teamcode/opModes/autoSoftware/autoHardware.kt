package org.firstinspires.ftc.teamcode.opModes.autoSoftware

import android.util.Size
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.EOCVWebcam
import org.firstinspires.ftc.teamcode.Enums.Alliance
import org.firstinspires.ftc.teamcode.Enums.AutoRandom
import org.firstinspires.ftc.teamcode.Enums.StartDist
import org.firstinspires.ftc.teamcode.Enums.StartSide
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.PIDVals
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.StartPose
import org.firstinspires.ftc.teamcode.extensions.BlinkExtensions.setPatternCo
import org.firstinspires.ftc.teamcode.extensions.Extensions.ledIND
import org.firstinspires.ftc.teamcode.extensions.MotorExtensions.runToPosition
import org.firstinspires.ftc.teamcode.extensions.MotorExtensions.runUsingEncoder
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.calcFlipPose
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig
import org.firstinspires.ftc.teamcode.opModes.camera.VPObjectDetect
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

//config can be enabled to change variables in real time through FTC Dash
//@Config
class autoHardware(opmode: LinearOpMode) // constructor
    : HardwareConfig(opmode) {
    lateinit var hardwareMap: HardwareMap // first initialization of the hardware map
    fun initAuto(ahwMap: HardwareMap, myOpMode: LinearOpMode, cycling: Boolean) {
        val telemetry: Telemetry =
            MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().telemetry)
        hardwareMap = ahwMap // hardware map initialization
        init(ahwMap, true) // hardware config initialization
        objProcessor = VPObjectDetect(StartPose.alliance)
        visionPortal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, EOCVWebcam.cam2_N))
            .setCameraResolution(Size(1280, 720))
            .addProcessors(objProcessor)
            .build()
        FtcDashboard.getInstance()
            .startCameraStream(objProcessor, 0.0) // start the camera stream on FTC Dash
        timer.reset()
        flipServo.calcFlipPose(80.0)
        lights.setPatternCo(RevBlinkinLedDriver.BlinkinPattern.RED)
        telemetry.update()
        if (myOpMode.isStopRequested) {
            return
        }
        green1.ledIND(red1, false)
        green2.ledIND(red2, false)
        green3.ledIND(red3, false)
        green4.ledIND(red4, false)
        ServoUtil.closeClaw(claw1)
        ServoUtil.closeClaw(claw2)
        myOpMode.waitForStart() // wait for the start button to be pressed
        rotationPIDF.setPIDF(
            PIDVals.rotationPIDFCo.p,
            PIDVals.rotationPIDFCo.i,
            PIDVals.rotationPIDFCo.d,
            PIDVals.rotationPIDFCo.f
        )
        extensionPIDF.setPIDF(
            PIDVals.extensionPIDFCo.p,
            PIDVals.extensionPIDFCo.i,
            PIDVals.extensionPIDFCo.d,
            PIDVals.extensionPIDFCo.f
        )
        lights.setPatternCo()
    }

    companion object {
        var START_POSE = Pose2d(0.0, 0.0, 0.0) // the start pose
        //    public static OpenCvWebcam webcam; // the webcam public we are using
        var blueRotate = -90 // final blue rotation
        var redRotate = 90 // final red rotation
        var startPose = Pose2d(12.0, -63.0, Math.toRadians(90.0))
        lateinit var spot: Pose2d // cycle position to be updated
        var autonomousRandom = AutoRandom.MID // default autonomous choice for spike mark
        lateinit var autoRandomReliable: AutoRandom // tracker for the AutoRandom enum
        var visionPortal: VisionPortal? = null // vision portal for the webcam
        var objProcessor: VPObjectDetect? = null // april tag processor for the vision portal

        fun getStartPose(alliance: Alliance, side: StartSide): Pose2d {
            StartPose.alliance = alliance
            StartPose.side = side
            when (alliance) {
                Alliance.RED -> {
                    when (side) {
                        StartSide.LEFT -> {
                            startDist = StartDist.LONG_SIDE
                            START_POSE = Pose2d(-36.0, -62.0, Math.toRadians(redRotate.toDouble()))
                            return Pose2d(-36.0, -62.0, Math.toRadians(redRotate.toDouble()))
                        }

                        StartSide.RIGHT -> {
                            startDist = StartDist.SHORT_SIDE
                            START_POSE = Pose2d(12.0, -62.0, Math.toRadians(redRotate.toDouble()))
                            return Pose2d(12.0, -62.0, Math.toRadians(redRotate.toDouble()))
                        }

                        else -> {}
                    }
                    when (side) {
                        StartSide.LEFT -> {
                            startDist = StartDist.SHORT_SIDE
                            START_POSE = Pose2d(12.0, 62.0, Math.toRadians(blueRotate.toDouble()))
                            return Pose2d(12.0, 62.0, Math.toRadians(blueRotate.toDouble()))
                        }

                        StartSide.RIGHT -> {
                            startDist = StartDist.LONG_SIDE
                            START_POSE = Pose2d(-36.0, 62.0, Math.toRadians(blueRotate.toDouble()))
                            return Pose2d(-36.0, 62.0, Math.toRadians(blueRotate.toDouble()))
                        }

                        else -> {}
                    }
                }

                Alliance.BLUE -> when (side) {
                    StartSide.LEFT -> {
                        startDist = StartDist.SHORT_SIDE
                        START_POSE = Pose2d(12.0, 62.0, Math.toRadians(blueRotate.toDouble()))
                        return Pose2d(12.0, 62.0, Math.toRadians(blueRotate.toDouble()))
                    }

                    StartSide.RIGHT -> {
                        startDist = StartDist.LONG_SIDE
                        START_POSE = Pose2d(-36.0, 62.0, Math.toRadians(blueRotate.toDouble()))
                        return Pose2d(-36.0, 62.0, Math.toRadians(blueRotate.toDouble()))
                    }

                    else -> {}
                }
            }
            return Pose2d(0.0, 0.0, 0.0)
        }

        // method to update the pose
        fun updatePose(drive: MecanumDrive) {
            PoseStorage.currentPose = drive.poseEstimate
        }

        // method to use encoders to go to a point with encoder
        fun encoderDrive(motor: DcMotor, position: Int, speed: Double, drive: MecanumDrive) {
            motor.runUsingEncoder()
            motor.targetPosition = motor.currentPosition + position
            drive.update()
            motor.runToPosition()
            motor.power = Math.abs(speed)
            while (motor.isBusy) {
                drive.update()
            }
            motor.power = 0.0
            //        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}
