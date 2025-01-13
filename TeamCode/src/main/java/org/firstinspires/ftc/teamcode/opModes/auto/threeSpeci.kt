package org.firstinspires.ftc.teamcode.opModes.auto

import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.customHardware.AutoHardware
import org.firstinspires.ftc.teamcode.customHardware.AutoHardware.Companion.redHuman
import org.firstinspires.ftc.teamcode.customHardware.AutoHardware.Companion.redSample
import org.firstinspires.ftc.teamcode.customHardware.AutoHardware.Companion.redSpecimen
import org.firstinspires.ftc.teamcode.customHardware.AutoHardware.Companion.redStartRight
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance

//@Autonomous(group = GroupingTitles.auto) //@Disabled//disabling the opmode
class threeSpeci(val alliance: Alliance) : LinearOpMode() {
    override fun runOpMode() { //if opmode is started
        val robot = AutoHardware(
            this,
            StartLocation(
                this.alliance,
                redStartRight,
            )
        )
        robot.once()
        waitForStart()

        if (opModeIsActive()) {
            robot.scorePreloadSpeci()
            robot.grabSpeci(redSpecimen)
            robot.placeSpeci(-3)
            robot.moveOneSpeci(
                Pose2d(
                    redSpecimen.position.x - 3,
                    redHuman.position.y + 25,
                    redSpecimen.heading.toDouble()
                )
            )
            robot.grabSpeci(
                Pose2d(
                    redSample.position.x - 16.0,
                    redSample.position.y,
                    redSample.heading.toDouble()
                ), true
            )
            robot.placeSpeci(-6)
            robot.endHuman()
        }
    }
}
