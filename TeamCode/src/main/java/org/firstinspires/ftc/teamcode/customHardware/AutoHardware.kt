package org.firstinspires.ftc.teamcode.customHardware

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartPose
import org.firstinspires.ftc.teamcode.customHardware.camera.camUtil.CameraUtilities.initializeProcessor
import org.firstinspires.ftc.teamcode.customHardware.camera.camUtil.PROCESSORS
import org.firstinspires.ftc.teamcode.extensions.BlinkExtensions.setPatternCo
import org.firstinspires.ftc.teamcode.extensions.PoseExtensions.toPose
import org.firstinspires.ftc.teamcode.followers.pedroPathing.follower.Follower

class AutoHardware(
    opmode: LinearOpMode,
    processor: PROCESSORS?,
    startLocation: StartLocation,
    ahwMap: HardwareMap = opmode.hardwareMap,
) : HardwareConfig(opmode, true) {

    private var startPose = StartPose(startLocation)
    var follower: Follower

    init {
        initRobot(ahwMap, true, startPose.pose)
        if (processor != null) {
            initializeProcessor(processor, ahwMap, CAM2, true)
        }
        follower = Follower(ahwMap)
        follower.setStartingPose(startPose.pose.toPose())
        telemetry.addData("Status", "Initialized")
        telemetry.addData("Alliance", startLocation.alliance)
        telemetry.addData("Start Side", startLocation.startSide)
        telemetry.update()
        opmode.waitForStart()
        timer.reset()
        lights.setPatternCo()
    }
}
