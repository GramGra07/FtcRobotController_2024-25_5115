package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.UtilClass.camUtil.CameraUtilities.aprilTag
import org.firstinspires.ftc.teamcode.UtilClass.camUtil.CameraUtilities.initializeProcessor
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.cam2_N
import org.firstinspires.ftc.teamcode.rr.MecanumDrive
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection

class LocalizationSubsystem(ahwMap: HardwareMap) {
    private var currentDetections: List<AprilTagDetection> = emptyList()
    private var numDetections: Int = 0

    init {
        initializeProcessor(null, ahwMap, cam2_N, true)
//        aprilTag =
//            AprilTagProcessor.Builder() // The following default settings are available to un-comment and edit as needed.
//                .setDrawAxes(false)
//                .setDrawCubeProjection(false)
//                .setDrawTagOutline(true)
//                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
//                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
//                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
//                .setLensIntrinsics(972.571, 972.571, 667.598, 309.012)
//                .build()
//
//        // Adjust Image Decimation to trade-off detection-range for detection-rate.
//        // eg: Some typical detection data using a Logitech C920 WebCam
//        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
//        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
//        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
//        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
//        // Note: Decimation can be changed on-the-fly to adapt during a match.
//        aprilTag.setDecimation(3.0F)
//        val builder = VisionPortal.Builder()
//        builder.setCamera(ahwMap.get(WebcamName::class.java, cam2_N))
//            .setCameraResolution(Size(1280, 720))
//        builder.addProcessor(aprilTag)
//        visionPortal = builder.build()
//        visionPortal.setProcessorEnabled(aprilTag, true)
//        startCameraStream(visionPortal)
    }

    private fun getDetections(): Map<String, Double?>? {
        currentDetections = aprilTag.detections
        numDetections = currentDetections.size
        var returnable: MutableMap<String, Double?>? = mutableMapOf()
        var robotX: Double?
        var robotY: Double?
        if (numDetections > 0) {
            if (numDetections == 1) {
                robotX = currentDetections[0].ftcPose.x
                robotY = currentDetections[0].ftcPose.y
            } else {
                var totalX = 0.0
                var totalY = 0.0
                for (detection in currentDetections) {
                    totalX += detection.ftcPose.x
                    totalY += detection.ftcPose.y
                }
                val avgX = totalX / numDetections
                val avgY = totalY / numDetections
                robotX = avgX
                robotY = avgY
            }
            returnable = mapOf("X" to robotX, "Y" to robotY).toMutableMap()
        }
        return returnable
    }

    private fun parseDetections(): Pair<Double, Double>? {
        val coolVar = getDetections() ?: return null
        val x = coolVar["X"] ?: return null
        val y = coolVar["Y"] ?: return null
        return Pair(x, y)
    }

    fun telemetry(telemetry: Telemetry) {
        val parsed = parseDetections()
        telemetry.addData("# AprilTags Detected", numDetections)

        // Step through the list of detections and display info for each one.
        for (detection in currentDetections) {
            if (detection.metadata != null && parsed != null) {
//                telemetry.addLine(
//                    String.format(
//                        "\n==== (ID %d) %s",
//                        detection.id,
//                        detection.metadata.name
//                    )
//                )
                telemetry.addLine(
                    String.format(
                        "XYZ %6.1f %6.1f %6.1f  (inch)",
                        parsed.first,
                        parsed.second,
                        detection.ftcPose.z
                    )
                )
//                telemetry.addLine(
//                    String.format(
//                        "PRY %6.1f %6.1f %6.1f  (deg)",
//                        detection.ftcPose.pitch,
//                        detection.ftcPose.roll,
//                        detection.ftcPose.yaw
//                    )
//                )
//                telemetry.addLine(
//                    String.format(
//                        "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)",
//                        detection.ftcPose.range,
//                        detection.ftcPose.bearing,
//                        detection.ftcPose.elevation
//                    )
//                )
            }
        }

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.")
//        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)")
//        telemetry.addLine("RBE = Range, Bearing & Elevation")
    }

    fun relocalize(drive: MecanumDrive) {
        val parsed = parseDetections()
        if (parsed != null) {
            drive.pose = Pose2d(
                Vector2d(parsed.first, parsed.second),
                Rotation2d(drive.pose.heading.real, drive.pose.heading.imag)
            )
        }
    }
}