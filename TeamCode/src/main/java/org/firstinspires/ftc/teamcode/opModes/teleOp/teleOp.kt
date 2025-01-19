package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles

//@TeleOp(group = GroupingTitles.tele) //@Disabled//disabling the opmode

class teleOp(val alliance: Alliance) : LinearOpMode() {
    //declaring the class
    override fun runOpMode() { //if opmode is started
        val robot = HardwareConfig(this, false, StartLocation(this.alliance)) //initializing the robot
        waitForStart()
        robot.once() //runs once
        while (opModeIsActive()) { //while the op mode is active
            robot.doBulk()
        }
    }
}