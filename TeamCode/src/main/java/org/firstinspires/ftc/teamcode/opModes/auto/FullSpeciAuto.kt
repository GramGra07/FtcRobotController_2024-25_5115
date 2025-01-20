package org.firstinspires.ftc.teamcode.opModes.auto

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.customHardware.AutoHardware
import org.firstinspires.ftc.teamcode.customHardware.AutoHardware.Companion.redStartRight
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance

//@Autonomous(group = GroupingTitles.auto) //@Disabled//disabling the opmode
class FullSpeciAuto(val alliance: Alliance) : LinearOpMode() {
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
            robot.moveAllSpeci()
            robot.grabSpeci(
            )
            robot.placeSpeci(-3)
//            robot.grabSpeci(
//                Pose2d(
//                    redSpecimen.position.x - 3,
//                    redSpecimen.position.y,
//                    redSpecimen.heading.toDouble()
//                )
//            )
//            robot.placeSpeci(-6)
            robot.endHuman(
            )
        }
    }
}
