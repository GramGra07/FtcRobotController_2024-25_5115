package org.firstinspires.ftc.teamcode.teleOp

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.HardwareConfig

@TeleOp(group = "a") //@Disabled//disabling the opmode

class teleOp : LinearOpMode() {
    //declaring the class
    override fun runOpMode() { //if opmode is started
        var robot = HardwareConfig(this, hardwareMap)
        waitForStart()
        while (opModeIsActive()) { //while the op mode is active
            robot.doBulk()
        }
    }
}