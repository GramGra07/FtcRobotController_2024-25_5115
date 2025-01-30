package org.firstinspires.ftc.teamcode.opModes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.customHardware.AutoHardware
import org.firstinspires.ftc.teamcode.customHardware.AutoHardware.Companion.redStartLeft
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles

@Autonomous(group = GroupingTitles.auto) //@Disabled//disabling the opmode
class oneSample() : LinearOpMode() {
    override fun runOpMode() { //if opmode is started
        val robot = AutoHardware(
            this,
            StartLocation(
                Alliance.RED,
                redStartLeft,
            )
        )

        robot.once()
        waitForStart()

        if (opModeIsActive()) {
//            robot.hiBask()
            robot.scorePreloadSample()
            robot.collapse()
        }
    }
}
