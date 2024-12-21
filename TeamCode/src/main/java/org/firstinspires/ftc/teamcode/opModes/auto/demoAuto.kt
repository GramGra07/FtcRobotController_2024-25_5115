package org.firstinspires.ftc.teamcode.opModes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.customHardware.AutoHardware
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.Point
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles

@Autonomous(group = GroupingTitles.auto) //@Disabled//disabling the opmode
class demoAuto : LinearOpMode() {
    override fun runOpMode() { //if opmode is started
        val robot = AutoHardware(
            this,
            StartLocation(
                Alliance.RED,
                Point(144 - 9.0, 72 + 8.0),
                Math.toRadians(0.0)
            )
        )
        robot.autoSetup()
        robot.once()
        waitForStart()

        robot.specimenAutoR.start()
        while (robot.specimenAutoR.mainLoop(this)) {
            robot.specimenAutoR.update()
        }
    }
}
