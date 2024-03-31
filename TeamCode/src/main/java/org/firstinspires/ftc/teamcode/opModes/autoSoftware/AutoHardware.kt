package org.firstinspires.ftc.teamcode.opModes.autoSoftware

import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.UtilClass.camUtil.CameraUtilities.initializeProcessor
import org.firstinspires.ftc.teamcode.UtilClass.camUtil.Processor
import org.firstinspires.ftc.teamcode.extensions.BlinkExtensions.setPatternCo
import org.firstinspires.ftc.teamcode.extensions.PoseExtensions.toStartPose
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.ledIND
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig
import org.firstinspires.ftc.teamcode.rr.MecanumDrive
import org.firstinspires.ftc.teamcode.startEnums.Alliance
import org.firstinspires.ftc.teamcode.startEnums.StartSide
import org.firstinspires.ftc.teamcode.storage.PoseStorage

//config can be enabled to change variables in real time through FTC Dash
//@Config
class AutoHardware(
    opmode: LinearOpMode,
    ahwMap: HardwareMap,
    alliance: Alliance,
    startSide: StartSide
) // constructor
    : HardwareConfig(opmode, ahwMap, true) {

    private var startPose = defaultStartPose.toStartPose()
    private lateinit var autoVars: HashMap<AutoVarEnums, Boolean>
    var drive: MecanumDrive

    init {
        initRobot(ahwMap, true)
        drive = MecanumDrive(ahwMap, getStartPose(alliance, startSide))
        initAutoVars(alliance, startSide)
        autoVars[AutoVarEnums.VISION_READY] =
            initializeProcessor(Processor.OBJECT_DETECT, ahwMap, cam2_N, true)
        green1.ledIND(red1, false)
        green2.ledIND(red2, false)
        green3.ledIND(red3, false)
        green4.ledIND(red4, false)
        while (!opmode.isStarted && !opmode.isStopRequested) {
            showAutoTelemetry()
        }
        timer.reset()
        lights.setPatternCo()
    }

    data class StartPose(
        val x: Double = defaultStartPose.position.x,
        val y: Double = defaultStartPose.position.y,
        val heading: Double = defaultStartPose.heading.toDouble()
    ) {

        private fun toPose(startPose: StartPose = StartPose(0.0, 0.0, 0.0)): Pose2d {
            return Pose2d(startPose.x, startPose.y, Math.toRadians(startPose.heading))
        }

        init {
            PoseStorage.currentPose = this.toPose()
        }
    }

    companion object {
        var defaultStartPose = Pose2d(12.0, -63.0, Math.toRadians(90.0))
    }

    private fun getStartPose(alliance: Alliance, startSide: StartSide): Pose2d {
        val spot = when (alliance) {
            Alliance.BLUE -> {
                when (startSide) {
                    StartSide.LEFT -> Pose2d(12.0, -63.0, Math.toRadians(90.0))
                    StartSide.RIGHT -> Pose2d(12.0, -63.0, Math.toRadians(90.0))
                }
            }

            Alliance.RED -> {
                when (startSide) {
                    StartSide.LEFT -> Pose2d(-12.0, -63.0, Math.toRadians(90.0))
                    StartSide.RIGHT -> Pose2d(-12.0, -63.0, Math.toRadians(90.0))
                }
            }
        }
        PoseStorage.currentPose = spot
        startPose = spot.toStartPose()
        return spot
    }

    enum class AutoVarEnums {
        RED_ALLIANCE,
        BLUE_ALLIANCE,
        LEFT_SIDE,
        RIGHT_SIDE,
        VISION_READY
    }

    private fun initAutoVars(alliance: Alliance, startSide: StartSide) {
        autoVars[AutoVarEnums.RED_ALLIANCE] = when (alliance) {
            Alliance.RED -> {
                true
            }

            Alliance.BLUE -> false
        }
        autoVars[AutoVarEnums.BLUE_ALLIANCE] = when (alliance) {
            Alliance.BLUE -> {
                true
            }

            Alliance.RED -> false
        }
        autoVars[AutoVarEnums.LEFT_SIDE] = when (startSide) {
            StartSide.LEFT -> {
                true
            }

            StartSide.RIGHT -> false
        }
        autoVars[AutoVarEnums.RIGHT_SIDE] = when (startSide) {
            StartSide.RIGHT -> {
                true
            }

            StartSide.LEFT -> false
        }
        autoVars[AutoVarEnums.VISION_READY] = false
    }

    private fun showAutoTelemetry() {
        if (autoVars[AutoVarEnums.RED_ALLIANCE] == true) {
            telemetry.addData("Alliance", "Red")
        } else if (autoVars[AutoVarEnums.BLUE_ALLIANCE] == true) {
            telemetry.addData("Alliance", "Blue")
        }
        if (autoVars[AutoVarEnums.LEFT_SIDE] == true) {
            telemetry.addData("Side", "Left")
        } else if (autoVars[AutoVarEnums.RIGHT_SIDE] == true) {
            telemetry.addData("Side", "Right")
        }
        if (autoVars[AutoVarEnums.VISION_READY] == true) {
            telemetry.addData("Vision", "Ready")
        }
        telemetry.update()
    }
}
