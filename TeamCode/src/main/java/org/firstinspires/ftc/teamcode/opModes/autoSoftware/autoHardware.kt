package org.firstinspires.ftc.teamcode.opModes.autoSoftware

import android.util.Size
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.EOCVWebcam.cam2_N
import org.firstinspires.ftc.teamcode.Enums.Alliance
import org.firstinspires.ftc.teamcode.Enums.AutoRandom
import org.firstinspires.ftc.teamcode.Enums.StartDist
import org.firstinspires.ftc.teamcode.Enums.StartSide
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.closeClaw
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.PIDVals.extensionPIDFCo
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.PIDVals.rotationPIDFCo
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.StartPose
import org.firstinspires.ftc.teamcode.extensions.BlinkExtensions.setPatternCo
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.ledIND
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
    var hardwareMap: HardwareMap? = null // first initialization of the hardware map
    fun initAuto(ahwMap: HardwareMap, myOpMode: LinearOpMode, cycling: Boolean) {
        val telemetry: Telemetry =
            MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().telemetry)
        hardwareMap = ahwMap // hardware map initialization
        init(ahwMap, true) // hardware config initialization
        objProcessor = VPObjectDetect(StartPose.alliance)
        //        if (aprilTagProcessor == null && cycling == true) {
//            aprilTagProcessor = new AprilTagProcessor.Builder()
//                    .setLensIntrinsics(972.571, 972.571, 667.598, 309.012)
//                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
//                    .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
//                    .setDrawAxes(false)
//                    .setDrawTagOutline(true)
//                    .setDrawTagID(true)
//                    .build();
//        }
        visionPortal = VisionPortal.Builder()
            .setCamera(ahwMap.get<WebcamName>(WebcamName::class.java, cam2_N))
            .setCameraResolution(Size(1280, 720))
            .addProcessors(objProcessor)
            .build()
        FtcDashboard.getInstance()
            .startCameraStream(objProcessor, 0.0) // start the camera stream on FTC Dash
        HardwareConfig.Companion.timer.reset()
        //        ServoUtil.closeClaw(HardwareConfig.claw1);
//        ServoUtil.closeClaw(HardwareConfig.claw2);
        flipServo.calcFlipPose(80.0)
        telemetry.update()
        if (myOpMode.isStopRequested) {
            return
        }
        green1.ledIND(red1, false)
        green2.ledIND(red2, false)
        green3.ledIND(red3, false)
        green4.ledIND(red4, false)
        closeClaw(claw1)
        closeClaw(claw2)
        myOpMode.waitForStart() // wait for the start button to be pressed
        HardwareConfig.Companion.rotationPIDF.setPIDF(
            rotationPIDFCo.p,
            rotationPIDFCo.i,
            rotationPIDFCo.d,
            rotationPIDFCo.f
        )
        HardwareConfig.Companion.extensionPIDF.setPIDF(
            extensionPIDFCo.p,
            extensionPIDFCo.i,
            extensionPIDFCo.d,
            extensionPIDFCo.f
        )
        //        visionPortal.setProcessorEnabled(objProcessor, false);
        lights.setPatternCo()
    }

    companion object {
        //    public static OpenCvWebcam webcam; // the webcam public we are using
        var START_POSE = Pose2d(0.0, 0.0, 0.0) // the start pose
        var blueRotate = -90 // final blue rotation
        var redRotate = 90 // final red rotation

        //default start position for RoadRunner
        @JvmField
        var startPose = Pose2d(12.0, -63.0, Math.toRadians(90.0))
        lateinit var spot: Pose2d // cycle position to be updated
        var autonomousRandom: AutoRandom =
            AutoRandom.MID // default autonomous choice for spike mark
        var autoRandomReliable: AutoRandom? = null // tracker for the AutoRandom enum
        var visionPortal: VisionPortal? = null // vision portal for the webcam
        var objProcessor: VPObjectDetect? = null // april tag processor for the vision portal
        var aprilTagProcessor: AprilTagProcessor? =
            null // april tag processor for the vision portal

        fun doAprilTagPoseCorrection(
            processor: AprilTagProcessor,
            telemetry: Telemetry,
            drive: MecanumDrive
        ) {
            val currentDetections: List<AprilTagDetection> = processor.getDetections()
            telemetry.addData("# AprilTags Detected", currentDetections.size)
            var pose = Pose2d(0.0, 0.0, drive.poseEstimate.heading)

            // Step through the list of detections and display info for each one.
            for (detection in currentDetections) {
                if (detection.metadata != null) {
                    if (detection.id == 7 || detection.id == 10) {
                        telemetry.addLine(
                            String.format(
                                "\n==== (ID %d) %s",
                                detection.id,
                                detection.metadata.name
                            )
                        )
                        telemetry.addLine(
                            String.format(
                                "XYZ %6.1f %6.1f %6.1f  (inch)",
                                detection.ftcPose.x,
                                detection.ftcPose.y,
                                detection.ftcPose.z
                            )
                        )
                        telemetry.addLine(
                            String.format(
                                "PRY %6.1f %6.1f %6.1f  (deg)",
                                detection.ftcPose.pitch,
                                detection.ftcPose.roll,
                                detection.ftcPose.yaw
                            )
                        )
                        telemetry.addLine(
                            String.format(
                                "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)",
                                detection.ftcPose.range,
                                detection.ftcPose.bearing,
                                detection.ftcPose.elevation
                            )
                        )
                        val aprilT = Pose2d(15.0, -72.0, Math.toRadians(0.0))
                        val aprilS = Pose2d(-15.0, -72.0, Math.toRadians(0.0))
                        var detectablePose: Pose2d
                        detectablePose = if (detection.id == 7) {
                            aprilS
                        } else {
                            aprilT
                        }
                        pose = Pose2d(
                            detectablePose.x - detection.ftcPose.y,
                            detectablePose.y - detection.ftcPose.x,
                            drive.poseEstimate.heading
                        )
                    }
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id))
                    telemetry.addLine(
                        String.format(
                            "Center %6.0f %6.0f   (pixels)",
                            detection.center.x,
                            detection.center.y
                        )
                    )
                }
            }
            if (pose.x != 0.0 && pose.y != 0.0) {
                telemetry.update()
                drive.poseEstimate = pose
            }
        }

        // shifts left or right depending on the random
        //    public static int fwd = 1;
        //    public static void shiftAuto(MecanumDrive drive) {
        //        if (startDist == StartDist.LONG_SIDE) {
        //            fwd = 5;d shiftAuto(MecanumDrive drive) {
        //        if (startDist == StartDist.LONG_SIDE) {
        //            fwd = 5;
        //        }
        //        if (autoRandomReliable == AutoRandom.left) {
        //            ShiftTrajectories.leftOffset = 4;
        //        }
        //        switch (autoRandomReliable) {
        //            case left:
        //                drive.followTrajectorySequence(ShiftTrajectories.shiftLeft(drive));
        //                break;
        //            case right:
        //                drive.followTrajectorySequence(ShiftTrajectories.shiftRight(drive));
        //                break;
        //        }
        //    }
        //        }
        //        if (autoRandomReliable == AutoRandom.left) {
        //            ShiftTrajectories.leftOffset = 4;
        //        }
        //        switch (autoRandomReliable) {
        //            case left:
        //                drive.followTrajectorySequence(ShiftTrajectories.shiftLeft(drive));
        //                break;
        //            case right:
        //                drive.followTrajectorySequence(ShiftTrajectories.shiftRight(drive));
        //                break;
        //        }
        //    }
        // method to get the start pose
        fun getStartPose(alliance: Alliance, side: StartSide): Pose2d {
            StartPose.alliance = alliance
            StartPose.side = side
            when (alliance) {
                Alliance.RED -> {
                    when (side) {
                        StartSide.LEFT -> {
                            HardwareConfig.Companion.startDist = StartDist.LONG_SIDE
                            START_POSE = Pose2d(-36.0, -62.0, Math.toRadians(redRotate.toDouble()))
                            Pose2d(-36.0, -62.0, Math.toRadians(redRotate.toDouble()))
                        }

                        StartSide.RIGHT -> {
                            HardwareConfig.Companion.startDist = StartDist.SHORT_SIDE
                            START_POSE = Pose2d(12.0, -62.0, Math.toRadians(redRotate.toDouble()))
                            Pose2d(12.0, -62.0, Math.toRadians(redRotate.toDouble()))
                        }
                    }
                }

                Alliance.BLUE -> {
                    when (side) {
                        StartSide.LEFT -> {
                            HardwareConfig.Companion.startDist = StartDist.SHORT_SIDE
                            START_POSE =
                                Pose2d(12.0, 62.0, Math.toRadians(blueRotate.toDouble()))
                            return Pose2d(12.0, 62.0, Math.toRadians(blueRotate.toDouble()))
                        }

                        StartSide.RIGHT -> {
                            HardwareConfig.Companion.startDist = StartDist.LONG_SIDE
                            START_POSE =
                                Pose2d(-36.0, 62.0, Math.toRadians(blueRotate.toDouble()))
                            return Pose2d(-36.0, 62.0, Math.toRadians(blueRotate.toDouble()))
                        }
                    }
                }
            }
            return START_POSE
        }

        // method to update the pose
        fun updatePose(drive: MecanumDrive) {
            PoseStorage.currentPose = drive.poseEstimate
        }
        // method to use encoders to go to a point with encoder
        fun encoderDrive(motor: DcMotor, position: Int, speed: Double, drive: MecanumDrive) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
            motor.setTargetPosition(motor.getCurrentPosition() + position)
            drive.update()
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION)
            motor.setPower(Math.abs(speed))
            while (motor.isBusy()) {
                drive.update()
            }
            motor.setPower(0.0)
            //        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }


}
