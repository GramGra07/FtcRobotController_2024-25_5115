package org.firstinspires.ftc.teamcode.opModes.auto

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.customHardware.AutoHardware
import org.firstinspires.ftc.teamcode.customHardware.AutoHardware.Companion.blueStartRight
import org.firstinspires.ftc.teamcode.customHardware.AutoHardware.Companion.redStartRight
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance

//@Autonomous(group = GroupingTitles.auto) //@Disabled//disabling the opmode
class FullAuto(val alliance: Alliance) : LinearOpMode() {
    override fun runOpMode() { //if opmode is started
        val robot = AutoHardware(
            this,
            StartLocation(
                this.alliance,
                if (alliance == Alliance.RED) redStartRight else blueStartRight,
            )
        )
//        var auto: StateMachine<out Enum<*>> = robot.fullSpecimenAutoR
//        if (alliance == Alliance.BLUE) {
//            auto = robot.fullSpecimenAutoB
//        }
        robot.autoSetup()
        robot.once()
        waitForStart()

//        auto.start()
//        while (auto.mainLoop(this)) {
//            auto.update()
//        }
    }
}
