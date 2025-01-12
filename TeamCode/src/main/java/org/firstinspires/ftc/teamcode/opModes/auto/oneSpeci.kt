package org.firstinspires.ftc.teamcode.opModes.auto

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.customHardware.AutoHardware
import org.firstinspires.ftc.teamcode.customHardware.AutoHardware.Companion.redStartRight
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance

//@Autonomous(group = GroupingTitles.auto) //@Disabled//disabling the opmode
class oneSpeci(val alliance: Alliance) : LinearOpMode() {
    override fun runOpMode() { //if opmode is started
        val robot = AutoHardware(
            this,
            StartLocation(
                this.alliance,
                redStartRight,
            )
        )
//        var auto: StateMachine<out Enum<*>> = robot.smallSpecimenAutoR
//        if (alliance == Alliance.BLUE) {
//            auto = robot.smallSpecimenAutoB
//        }
        robot.autoSetup()
        robot.once()
        waitForStart()

        if (opModeIsActive()) {
            robot.scorePreloadSpeci()
            robot.endHuman()
        }
    }
}
