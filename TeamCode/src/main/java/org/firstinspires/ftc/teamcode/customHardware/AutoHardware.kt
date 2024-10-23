package org.firstinspires.ftc.teamcode.customHardware

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.extensions.BlinkExtensions.setPatternCo
import org.firstinspires.ftc.teamcode.extensions.PoseExtensions.toPose
import org.firstinspires.ftc.teamcode.followers.pedroPathing.follower.Follower
import org.firstinspires.ftc.teamcode.utilClass.storage.GameStorage

class AutoHardware(
    opmode: LinearOpMode,
    startLocation: StartLocation,
    ahwMap: HardwareMap = opmode.hardwareMap,
) : HardwareConfig(opmode, true, startLocation) {

    var follower: Follower

    init {
        initRobot(ahwMap, true, startLocation)
        GameStorage.alliance = startLocation.alliance
        follower = Follower(ahwMap)
        follower.setStartingPose(startLocation.startPose.toPose())
        telemetry.addData("Status", "Initialized")
        telemetry.addData("Alliance", startLocation.alliance)
        telemetry.update()
        opmode.waitForStart()
        timer.reset()
        lights.setPatternCo()
    }
}
