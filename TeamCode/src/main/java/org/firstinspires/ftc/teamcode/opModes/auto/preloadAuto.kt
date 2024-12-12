package org.firstinspires.ftc.teamcode.opModes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.Point
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles

@Autonomous(group = GroupingTitles.auto) //@Disabled//disabling the opmode
//@Disabled
class preloadAuto : LinearOpMode() {
    //declaring the class
    override fun runOpMode() { //if opmode is started
        val robot = HardwareConfig(
            this,
            true,
            StartLocation(Alliance.RED, Point(0.0, 0.0), 0.0)
        )
        waitForStart()
    }
}
