package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Point
import org.firstinspires.ftc.teamcode.UtilClass.camUtil.ATLocations.Companion.getLocation
import org.firstinspires.ftc.teamcode.UtilClass.camUtil.CameraUtilities.aprilTag
import org.firstinspires.ftc.teamcode.UtilClass.camUtil.CameraUtilities.initializeProcessor
import org.firstinspires.ftc.teamcode.UtilClass.camUtil.Processor
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.cam2_N
import org.firstinspires.ftc.teamcode.rr.MecanumDrive
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection

class LocalizationSubsystem(ahwMap: HardwareMap) {
    private var currentDetections: List<AprilTagDetection> = emptyList()
    private var numDetections: Int = 0

    init {
        initializeProcessor(Processor.APRIL_TAG, ahwMap, cam2_N, true)
    }

    private fun getDetections(): Map<String, Double?>? {
        currentDetections = aprilTag.detections
        val id: Int
        numDetections = currentDetections.size
        var returnable: MutableMap<String, Double?>? = mutableMapOf()
        var poseX: Double?
        var poseY: Double?
        if (numDetections > 0) {
            if (numDetections == 1) {
                id = currentDetections[0].id
                poseX = currentDetections[0].ftcPose.x
                poseY = currentDetections[0].ftcPose.y
            } else {
                var totalX = 0.0
                var totalY = 0.0
                for (detection in currentDetections) {
                    totalX += detection.ftcPose.x
                    totalY += detection.ftcPose.y
                }
                val avgX = totalX / numDetections
                val avgY = totalY / numDetections
                poseX = avgX
                poseY = avgY
                id = currentDetections[numDetections - 1].id
            }
            val correctedPose = getCorrectedPose(getLocation(id), poseX, poseY)
            poseX = correctedPose.x
            poseY = correctedPose.y
            returnable = mapOf("X" to poseX, "Y" to poseY).toMutableMap()
        }
        return returnable
    }

    private fun getCorrectedPose(point: Point, x: Double, y: Double): Point {
        val newX = point.x!! - x
        val newY = point.y!! - y
        return Point(newX, newY)
    }

    private fun parseDetections(): Pair<Double, Double>? {
        val coolVar = getDetections() ?: return null
        val x = coolVar["X"] ?: return null
        val y = coolVar["Y"] ?: return null
        return Pair(y, x)
    }

    fun telemetry(telemetry: Telemetry) {
        val parsed = parseDetections()
        telemetry.addData("# AprilTags Detected", numDetections)
        if (parsed != null) {
            telemetry.addLine(
                String.format(
                    "XYZ %6.1f %6.1f %6.1f  (inch)",
                    parsed.first,
                    parsed.second,
                    //                        detection.ftcPose.z
                )
            )
        }
        // Step through the list of detections and display info for each one.
        for (detection in currentDetections) {
            if (detection.metadata != null && parsed != null) {
                telemetry.addLine(
                    String.format(
                        "\n==== (ID %d) %s",
                        detection.id,
                        detection.metadata.name
                    )
                )
            }
        }
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.")
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