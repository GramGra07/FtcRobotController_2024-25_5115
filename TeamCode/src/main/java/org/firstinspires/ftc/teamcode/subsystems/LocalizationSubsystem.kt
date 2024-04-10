package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Point
import org.firstinspires.ftc.teamcode.UtilClass.camUtil.ATLocations
import org.firstinspires.ftc.teamcode.UtilClass.camUtil.ATLocations.Companion.getLocation
import org.firstinspires.ftc.teamcode.UtilClass.camUtil.CameraUtilities.aprilTag
import org.firstinspires.ftc.teamcode.UtilClass.camUtil.CameraUtilities.initializeProcessor
import org.firstinspires.ftc.teamcode.UtilClass.camUtil.Processor
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.cam1_N
import org.firstinspires.ftc.teamcode.rr.MecanumDrive
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import kotlin.math.abs

class LocalizationSubsystem(ahwMap: HardwareMap) {
    private var currentDetections: List<AprilTagDetection> = emptyList()
    private var numDetections: Int = 0
    private var parsedData: Pair<Double, Double>? = null
    private var currentSeenID: MutableList<Int>? = mutableListOf()
    private var localizingID: MutableList<Int>? = mutableListOf()

    init {
        initializeProcessor(Processor.APRIL_TAG, ahwMap, cam1_N, true)
    }

    private fun getDetections(): Map<String, Double?>? {
        currentDetections = aprilTag.detections
        currentSeenID = mutableListOf()
        localizingID = mutableListOf()
        if (currentDetections.isNotEmpty()) {
            for (detection in currentDetections) {
                currentSeenID!!.add(detection.id)
            }
        }
        val id: Int
        numDetections = currentDetections.size
        var returnable: MutableMap<String, Double?>? = mutableMapOf()
        var poseX: Double?
        var poseY: Double?
        var correctedPose: Point
        val xThresh = 24 // inches
        val yThresh = 36
        if (numDetections > 0 && (currentDetections[0].id != 7) &&
            (((currentSeenID!!.contains(5)) || (currentSeenID!!.contains(8))) || (currentSeenID!!.contains(
                2
            ) || (currentSeenID!!.contains(9))))
        ) {
            if ((numDetections == 1)) {
                id = currentDetections[0].id
                poseX = currentDetections[0].ftcPose.x
                poseY = currentDetections[0].ftcPose.y
                if (abs(poseX)<xThresh || abs(poseY)<yThresh) {
                    localizingID!!.add(id)
                    correctedPose = getCorrectedPose(getLocation(id - 1), poseX, poseY, id - 1)
                    poseX = correctedPose.x
                    poseY = correctedPose.y
                    returnable = mapOf("X" to poseX, "Y" to poseY).toMutableMap()
                }
            } else {
                val id: Int
                for (detection in currentDetections) {
                    if (detection.id == 5 || detection.id == 8 || detection.id == 2 || detection.id == 9) {
                        id = detection.id
                        poseX = detection.ftcPose.x
                        poseY = detection.ftcPose.y
                        if (abs(poseX)<xThresh || abs(poseY)<yThresh) {
                            localizingID!!.add(id)
                            correctedPose = getCorrectedPose(getLocation(id), poseX, poseY, id)
                            poseX = correctedPose.x
                            poseY = correctedPose.y
                            returnable = mapOf("X" to poseX, "Y" to poseY).toMutableMap()
                        }
                        break
                    }
                }
            }
        }
        return returnable
    }

    private fun getCorrectedPose(point: Point, x: Double, y: Double, id: Int): Point {
        var newX: Double
        var newY: Double
        val xDirOnRobot = 6
        val yDirOnRobot = 7
        if (id in 1..6) {
            newX = point.x!! + x
            newY = point.y!! - y
            newX += xDirOnRobot
            newY -= yDirOnRobot
        } else {
            newX = point.x!! + x
            newY = point.y!! + y
            newX -= xDirOnRobot
            newY += yDirOnRobot
        }
        return Point(newX, newY)
    }

    private fun parseDetections(): Pair<Double, Double>? {
        val coolVar = getDetections() ?: return null
        val x = coolVar["X"] ?: return null
        val y = coolVar["Y"] ?: return null
        parsedData = Pair(y, x)
        return Pair(y, x)
    }

    fun telemetry(telemetry: Telemetry) {
//        val parsed = parsedData
//        telemetry.addData("# AprilTags Detected", numDetections)
//        if (parsed != null) {
//            telemetry.addLine(
//                String.format(
//                    "XYZ %6.1f %6.1f (inch)",
//                    parsed.first,
//                    parsed.second,
//                    //                        detection.ftcPose.z
//                )
//            )
//        }
//        // Step through the list of detections and display info for each one.
//        for (detection in currentDetections) {
//            if (detection.metadata != null && parsed != null) {
//                telemetry.addLine(
//                    String.format(
//                        "\n==== (ID %d) %s",
//                        detection.id,
//                        detection.metadata.name
//                    )
//                )
//            }
//        }
        val currentSeenID = currentSeenID
        telemetry.addData("IDS", currentSeenID)
    }

    fun draw(packet: TelemetryPacket) {
        for (id in ATLocations.allLocations) {
            packet.fieldOverlay().setStroke("blue").setAlpha(0.5)
                .strokeRect(id.second.location.y!!, id.second.location.x!!, 0.5, 0.5)
        }
        val currentSeenID = currentSeenID
        if (!currentSeenID.isNullOrEmpty()) {
            for (id in currentSeenID) {
                val point = getLocation(id - 1)
                packet.fieldOverlay().setAlpha(1.0).setStroke("green")
                    .strokeRect(point.y!!, point.x!!, 1.0, 1.0)
            }
        }
        val localizingID = localizingID
        if (!localizingID.isNullOrEmpty()) {
            for (id in localizingID) {
                val point = getLocation(id - 1)
                packet.fieldOverlay().setAlpha(1.0).setStroke("orange")
                    .strokeRect(point.y!!, point.x!!, 1.0, 1.0)
            }
        }
    }

    fun relocalize(drive: MecanumDrive) {
        parseDetections()
        val parsed = parsedData
        if (parsed != null) {
            drive.pose = Pose2d(
                Vector2d(parsed.first, parsed.second),
                Rotation2d(drive.pose.heading.real, drive.pose.heading.imag)
            )
            parsedData = null
        }
    }
}