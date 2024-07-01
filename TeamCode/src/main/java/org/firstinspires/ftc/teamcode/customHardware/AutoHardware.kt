package org.firstinspires.ftc.teamcode.customHardware

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.customHardware.autoHardware.AutoVarEnums
import org.firstinspires.ftc.teamcode.customHardware.autoHardware.StartLocation
import org.firstinspires.ftc.teamcode.customHardware.autoHardware.StartPose
import org.firstinspires.ftc.teamcode.extensions.BlinkExtensions.setPatternCo
import org.firstinspires.ftc.teamcode.rr.MecanumDrive
import org.firstinspires.ftc.teamcode.startEnums.Alliance
import org.firstinspires.ftc.teamcode.startEnums.StartSide
import org.firstinspires.ftc.teamcode.customHardware.camera.camUtil.CameraUtilities.initializeProcessor
import org.firstinspires.ftc.teamcode.customHardware.camera.camUtil.Processor

class AutoHardware(
    opmode: LinearOpMode,
    processor: Processor?,
    startLocation: StartLocation,
    ahwMap: HardwareMap = opmode.hardwareMap,
) : HardwareConfig(opmode, true) {

    private var startPose = StartPose(startLocation)
    private var autoVars: HashMap<AutoVarEnums, Boolean> = hashMapOf()
    var drive: MecanumDrive

    init {
        initRobot(ahwMap, true)
        initAutoVars(startPose.startLocation)
        drive = MecanumDrive(ahwMap, startPose.pose)
        autoVars[AutoVarEnums.VISION_READY] =
            initializeProcessor(processor, ahwMap, CAM2, true)

        showAutoTelemetry()
        opmode.waitForStart()
        timer.reset()
        lights.setPatternCo()
    }

    private fun initAutoVars(startLocation: StartLocation) {
        val alliance = startLocation.alliance
        val startSide = startLocation.startSide
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
