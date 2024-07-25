package org.firstinspires.ftc.teamcode.customHardware.camera.camUtil

import android.util.Size
import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig.Companion.lights
import org.firstinspires.ftc.teamcode.customHardware.camera.Camera
import org.firstinspires.ftc.teamcode.customHardware.camera.CameraType
import org.firstinspires.ftc.teamcode.customHardware.camera.LensIntrinsics
import org.firstinspires.ftc.teamcode.extensions.BlinkExtensions.setPatternCo
import org.firstinspires.ftc.teamcode.vision.VPObjectDetect
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.VisionProcessor
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.gentrifiedApps.velocityvision.enums.CSpace
import org.gentrifiedApps.velocityvision.pipelines.moa.AssumedBuilder
import org.gentrifiedApps.velocityvision.pipelines.moa.DetectionBuilder
import org.gentrifiedApps.velocityvision.pipelines.moa.MeanColorOfAreaDetector
import org.opencv.core.Point
import org.opencv.core.Rect
import org.opencv.core.Scalar

object CameraUtilities {
    private fun startCameraStream(streamSource: CameraStreamSource) {
        FtcDashboard.getInstance()
            .startCameraStream(streamSource, 0.0)
    }

    fun stopCameraStream() {
        FtcDashboard.getInstance()
            .stopCameraStream()
    }

    var mainCamera: Camera = setupCameras(CameraType.LOGITECH)//!CHANGE

    private var runningProcessors: MutableList<VisionProcessor> =
        emptyList<VisionProcessor>().toMutableList()
    private lateinit var visionPortal: VisionPortal
    lateinit var aprilTag: AprilTagProcessor
    private lateinit var objProcessor: VPObjectDetect
    private lateinit var pubProcessor: MeanColorOfAreaDetector

    fun initializeProcessor(
        processor: Processor?,
        ahwMap: HardwareMap,
        camera: String,
        ftcDashboard: Boolean
    ): Boolean {
        when (processor) {
            null -> {}
            Processor.APRIL_TAG -> {
                aprilTag =
                    AprilTagProcessor.Builder() // The following default settings are available to un-comment and edit as needed.
                        .setDrawAxes(false)
                        .setDrawCubeProjection(false)
                        .setDrawTagOutline(true)
                        .setDrawAxes(true)
                        .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                        .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                        .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                        //                    .setLensIntrinsics(972.571, 972.571, 667.598, 309.012)
                        .setLensIntrinsics(
                            mainCamera.lensIntrinsics.fx, mainCamera.lensIntrinsics.fy,
                            mainCamera.lensIntrinsics.cx, mainCamera.lensIntrinsics.cy
                        )
                        .build()

                // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
                // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
                // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
                // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
                aprilTag.setDecimation(3.0F)
                runningProcessors.add(aprilTag)
            }

            Processor.OBJECT_DETECT -> {
                objProcessor = VPObjectDetect()
                runningProcessors.add(objProcessor)
            }

            Processor.PUB_TEST -> {
                pubProcessor = MeanColorOfAreaDetector(
                    CSpace.YCrCb,
                    DetectionBuilder(
                        Rect(Point(120.0, 50.0), Point(230.0, 150.0)), "middle",
                        Scalar(0.0, 140.0, 0.0),
                        Scalar(255.0, 255.0, 255.0)
                    ) { lights.setPatternCo(RevBlinkinLedDriver.BlinkinPattern.CONFETTI) },
                    DetectionBuilder(
                        Rect(Point(570.0, 70.0), Point(680.0, 170.0)), "right",
                        Scalar(0.0, 140.0, 0.0),
                        Scalar(255.0, 255.0, 255.0)
                    ) { lights.setPatternCo(RevBlinkinLedDriver.BlinkinPattern.WHITE) },
                    AssumedBuilder("left") { lights.setPatternCo(RevBlinkinLedDriver.BlinkinPattern.GOLD) }
                )
                runningProcessors.add(pubProcessor)
            }
        }
        if (processor != null) {
            val builder = VisionPortal.Builder()
            builder.setCamera(ahwMap.get(WebcamName::class.java, camera))
                .setCameraResolution(mainCamera.size)
            if (runningProcessors.size > 1) {
                builder.setLiveViewContainerId(0)
            }
            when (processor) {
                Processor.APRIL_TAG -> {
                    builder.addProcessor(aprilTag)
                }

                Processor.OBJECT_DETECT -> {
                    builder.addProcessor(objProcessor)
                }

                Processor.PUB_TEST -> {
                    builder.addProcessor(pubProcessor)
                }
            }
            visionPortal = builder.build()
            if (ftcDashboard) {
                startCameraStream(visionPortal)
            }
            return true
        } else {
            return false
        }
    }

    private fun setupCameras(cameraType: CameraType): Camera {
        return when (cameraType) {
            CameraType.ARDU_CAM -> {
                val ArduCam =
                    Camera("ArduCam",Size(1280, 720), LensIntrinsics(972.571, 972.571, 667.598, 309.012))
                ArduCam
            }

            CameraType.LOGITECH -> {
                val LogiC270 =
                    Camera("Logitech",Size(640, 480), LensIntrinsics(397.606, 397.606, 320.023, 239.979))
                LogiC270
            }
        }
    }
}