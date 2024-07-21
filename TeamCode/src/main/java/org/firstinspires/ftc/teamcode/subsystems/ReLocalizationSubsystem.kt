package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig.Companion.CAM1
import org.firstinspires.ftc.teamcode.customHardware.camera.camUtil.ATLocations
import org.firstinspires.ftc.teamcode.customHardware.camera.camUtil.ATLocations.Companion.getLocation
import org.firstinspires.ftc.teamcode.customHardware.camera.camUtil.CameraUtilities.aprilTag
import org.firstinspires.ftc.teamcode.customHardware.camera.camUtil.CameraUtilities.initializeProcessor
import org.firstinspires.ftc.teamcode.customHardware.camera.camUtil.Processor
import org.firstinspires.ftc.teamcode.utilClass.objects.Point
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import kotlin.math.abs

class ReLocalizationSubsystem(ahwMap: HardwareMap) {
    private var currentDetections: List<AprilTagDetection> = emptyList()
    private var numDetections: Int = 0
    private var parsedData: Pair<Double, Double>? = null
    private var currentSeenID: MutableList<Int>? = mutableListOf()
    private var localizingID: MutableList<Int>? = mutableListOf()
    private val acceptableIDs = listOf(5, 8, 2, 9)
    private val rejectedIDs = listOf(7)

    init {
        initializeProcessor(Processor.APRIL_TAG, ahwMap, CAM1, true)
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
        var xDif: Double?
        var yDif: Double?
        val correctedPose: Point
        val xThresh = 24
        val yThresh = 36
        if (numDetections > 0 &&
            rejectedIDs.any { currentSeenID?.contains(it) == false } &&
            acceptableIDs.any { currentSeenID?.contains(it) == true }
        ) {
            if ((numDetections == 1)) {
                id = currentDetections[0].id
                xDif = currentDetections[0].ftcPose.x
                yDif = currentDetections[0].ftcPose.y
                if (abs(xDif) < xThresh || abs(yDif) < yThresh) {
                    localizingID!!.add(id)
                    correctedPose = getCorrectedPose(getLocation(id - 1), xDif, yDif, id - 1)
                    xDif = correctedPose.x
                    yDif = correctedPose.y
                    returnable = mapOf("X" to xDif, "Y" to yDif).toMutableMap()
                }
            } else {
                val detection =
                    currentDetections.find { acceptableIDs.contains(it.id) }
                        ?: return null
                id = detection.id
                xDif = detection.ftcPose.x
                yDif = detection.ftcPose.y
                if (abs(xDif) < xThresh || abs(yDif) < yThresh) {
                    localizingID!!.add(id)
                    correctedPose = getCorrectedPose(getLocation(id), xDif, yDif, id)
                    xDif = correctedPose.x
                    yDif = correctedPose.y
                    returnable = mapOf("X" to xDif, "Y" to yDif).toMutableMap()
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
        val currentSeenID = currentSeenID
        telemetry.addData("IDS", currentSeenID)
    }

    fun draw(packet: TelemetryPacket) {
        val fieldOverlay = packet.fieldOverlay()
        ATLocations.allLocations.forEach { (id, locationData) ->
            if (localizingID!!.contains(id)) {
                fieldOverlay.setStroke("green").setAlpha(1.0)
            } else if (currentSeenID!!.contains(id)) {
                fieldOverlay.setStroke("orange").setAlpha(1.0)
            } else {
                val location = locationData.location
                fieldOverlay.setStroke("blue").setAlpha(0.5)
                fieldOverlay.strokeRect(location.y!!, location.x!!, 0.5, 0.5)
            }
        }
//
//        for (id in ATLocations.allLocations) {
//            packet.fieldOverlay().setStroke("blue").setAlpha(0.5)
//                .strokeRect(id.second.location.y!!, id.second.location.x!!, 0.5, 0.5)
//        }
//        val currentSeenID = currentSeenID
//        if (!currentSeenID.isNullOrEmpty()) {
//            for (id in currentSeenID) {
//                val point = getLocation(id - 1)
//                packet.fieldOverlay().setAlpha(1.0).setStroke("green")
//                    .strokeRect(point.y!!, point.x!!, 1.0, 1.0)
//            }
//        }
//        val localizingID = localizingID
//        if (!localizingID.isNullOrEmpty()) {
//            for (id in localizingID) {
//                val point = getLocation(id - 1)
//                packet.fieldOverlay().setAlpha(1.0).setStroke("orange")
//                    .strokeRect(point.y!!, point.x!!, 1.0, 1.0)
//            }
//        }
    }

    fun relocalize(localizerSubsystem: LocalizerSubsystem) {
        parseDetections()
        val parsed = parsedData
        if (parsed != null) {
            localizerSubsystem.setPose(
                Pose2d(
                    Vector2d(parsed.first, parsed.second),
                    localizerSubsystem.heading()
                )
            )
        }
    }
}
