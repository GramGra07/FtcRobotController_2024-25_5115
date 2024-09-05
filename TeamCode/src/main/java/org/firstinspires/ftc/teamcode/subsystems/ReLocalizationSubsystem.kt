package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig.Companion.CAM1
import org.firstinspires.ftc.teamcode.customHardware.camera.camUtil.AT.ATLocations.Companion.getLocation
import org.firstinspires.ftc.teamcode.customHardware.camera.camUtil.CameraUtilities
import org.firstinspires.ftc.teamcode.customHardware.camera.camUtil.CameraUtilities.aprilTag
import org.firstinspires.ftc.teamcode.customHardware.camera.camUtil.CameraUtilities.initializeProcessor
import org.firstinspires.ftc.teamcode.customHardware.camera.camUtil.PROCESSORS
import org.firstinspires.ftc.teamcode.utilClass.objects.Point
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import kotlin.math.abs

class ReLocalizationSubsystem(ahwMap: HardwareMap) {
    private var currentDetections: List<AprilTagDetection> = emptyList()
    private var numDetections: Int = 0
    private var parsedData: Pair<Double, Double>? = null

    companion object {
        var currentSeenID: MutableList<Int> = mutableListOf()
        var localizingID: MutableList<Int> = mutableListOf()
    }

    private val acceptableIDs = listOf(5, 8, 2, 9)

    init {
        initializeProcessor(PROCESSORS.APRIL_TAG, ahwMap, CAM1, true)
    }

    private fun getDetections(): Map<String, Double?>? {
        currentDetections = aprilTag.detections
        currentSeenID.clear()
        localizingID.clear()
        numDetections = currentDetections.size
        if (numDetections == 0) return null
        for (detection in currentDetections) {
            currentSeenID.add(detection.id)
        }
        val id: Int
        var xDif: Double?
        var yDif: Double?
        val correctedPose: Point
        val xThresh = 24
        val yThresh = 36
        if (acceptableIDs.any { it in currentSeenID }) {
            val detection = if (numDetections == 1) {
                currentDetections[0]
            } else {
                currentDetections.find { acceptableIDs.contains(it.id) } ?: return null
            }
            xDif = detection.ftcPose.x
            yDif = detection.ftcPose.y
            if (abs(xDif) >= xThresh && abs(yDif) >= yThresh) return null
            id = detection.id
            val detectionID = if (numDetections == 1) {
                id - 1
            } else {
                id
            }
            localizingID.add(id)
            correctedPose = getCorrectedPose(getLocation(detectionID), xDif, yDif, detectionID)
            xDif = correctedPose.x
            yDif = correctedPose.y
            return mapOf("X" to xDif, "Y" to yDif).toMutableMap()
        } else {
            return null
        }
    }

    private fun getCorrectedPose(point: Point, x: Double, y: Double, id: Int): Point {
        val offsetPoint = Pair(6.0, 7.0)
        val (offsetX, offsetY) = if (id in 1..6) {
            Pair(offsetPoint.first, -offsetPoint.second)
        } else {
            Pair(-offsetPoint.first, offsetPoint.second)
        }
        return Point(point.x!! + x + offsetX, point.y!! + y + offsetY)
//        var newX: Double
//        var newY: Double
//        val xOffsetOnRobot = 6
//        val yOffsetOnRobot = 7
//        if (id in 1..6) {
//            newX = point.x!! + x
//            newY = point.y!! - y
//            newX += xOffsetOnRobot
//            newY -= yOffsetOnRobot
//        } else {
//            newX = point.x!! + x
//            newY = point.y!! + y
//            newX -= xOffsetOnRobot
//            newY += yOffsetOnRobot
//        }
//        return Point(newX, newY)
    }

    private fun parseDetections(): Pair<Double, Double>? {
        val coolVar = getDetections() ?: return null
        val x = coolVar["X"] ?: return null
        val y = coolVar["Y"] ?: return null
        parsedData = Pair(y, x)
        return Pair(y, x)
    }

    fun telemetry(telemetry: Telemetry) {
        telemetry.addData("RELOCALIZATION", "")
        val mainCamera = CameraUtilities.mainCamera
        telemetry.addData("Camera", mainCamera.name)
        val currentSeenID = currentSeenID
        telemetry.addData("IDS", currentSeenID)
        val localizingID = localizingID
        telemetry.addData("Localizing ID", localizingID)
    }

//    fun draw(packet: TelemetryPacket) {
//        val fieldOverlay = packet.fieldOverlay()
//        ATLocations.allLocations.forEach { (id, locationData) ->
//            val location = locationData.location
//            if (localizingID!!.contains(id)) {
//                fieldOverlay.setStroke("green").setAlpha(1.0)
//            } else if (currentSeenID!!.contains(id)) {
//                fieldOverlay.setStroke("orange").setAlpha(1.0)
//            } else {
//                fieldOverlay.setStroke("blue").setAlpha(0.5)
//            }
//            fieldOverlay.strokeRect(location.y!!, location.x!!, 0.5, 0.5)
//        }
//    }

    fun relocalize(localizerSubsystem: LocalizerSubsystem) {
        val parsed = parseDetections()
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
