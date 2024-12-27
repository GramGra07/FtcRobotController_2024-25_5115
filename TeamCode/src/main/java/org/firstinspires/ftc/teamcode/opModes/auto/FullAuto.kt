package org.firstinspires.ftc.teamcode.opModes.auto

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.customHardware.AutoHardware
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.Point

//@Autonomous(group = GroupingTitles.auto) //@Disabled//disabling the opmode
class FullAuto(val alliance: Alliance) : LinearOpMode() {
    override fun runOpMode() { //if opmode is started
        val robot = AutoHardware(
            this,
            StartLocation(
                this.alliance,
                if (alliance == Alliance.RED) Point(135.0, 80.0) else Point(9.0, 80.0),
                if (alliance == Alliance.RED) Math.toRadians(0.0) else Math.toRadians(180.0)
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
