package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig

@TeleOp(group = "a") //@Disabled//disabling the opmode

class teleOp : LinearOpMode() {
    //declaring the class
    var robot = HardwareConfig(this, hardwareMap, false)
    override fun runOpMode() { //if opmode is started
        waitForStart()
        while (opModeIsActive()) { //while the op mode is active
            robot.doBulk()
        }
    }
}