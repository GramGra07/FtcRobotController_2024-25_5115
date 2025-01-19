package org.firstinspires.ftc.teamcode.opModes.auto

import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.customHardware.AutoHardware
import org.firstinspires.ftc.teamcode.customHardware.AutoHardware.Companion.redNeutralSample
import org.firstinspires.ftc.teamcode.customHardware.AutoHardware.Companion.redStartLeft
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance

//@Autonomous(group = GroupingTitles.auto) //@Disabled//disabling the opmode
class FullSampleAuto(val alliance: Alliance) : LinearOpMode() {
    override fun runOpMode() { //if opmode is started
        val robot = AutoHardware(
            this,
            StartLocation(
                this.alliance,
                redStartLeft,
            )
        )
        robot.once()
        waitForStart()

        if (opModeIsActive()) {
            robot.scorePreloadSample()
            robot.getSampleR()
            robot.scoreSample(

            )
            robot.getSampleM()
            robot.scoreSample(

            )
            robot.getSampleL()
            robot.scoreSample(

            )
            robot.end()
        }
    }
}
