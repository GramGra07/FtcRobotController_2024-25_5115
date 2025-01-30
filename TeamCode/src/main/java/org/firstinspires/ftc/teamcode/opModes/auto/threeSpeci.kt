package org.firstinspires.ftc.teamcode.opModes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.customHardware.AutoHardware
import org.firstinspires.ftc.teamcode.customHardware.AutoHardware.Companion.redStartRight
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles

@Autonomous(group = GroupingTitles.auto) //@Disabled//disabling the opmode
@Disabled
class threeSpeci() : LinearOpMode() {
    override fun runOpMode() { //if opmode is started
        val robot = AutoHardware(
            this,
            StartLocation(
                Alliance.RED,
                redStartRight,
            )
        )
        robot.once()
        waitForStart()

        if (opModeIsActive()) {
            robot.scorePreloadSpeci()
            robot.grabSpeci()
            robot.placeSpeci(-3)
            robot.moveOneSpeci(
            )
            robot.grabSpeci(
                true
            )
            robot.placeSpeci(-6)
            robot.endHuman()
        }
    }
}
