package org.firstinspires.ftc.teamcode.opModes.testers

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig

@TeleOp(group = "a") 
//@Disabled//disabling the opmode

class OTOSTester : LinearOpMode() {
    //declaring the class
    override fun runOpMode() { //if opmode is started
        val sensor=initOTOS(this.hardwareMap,Pose2d(0,0,0),"spark")
        waitForStart()
        
        while (opModeIsActive()) { 
sensor.telemetry(telemetry)
        }
    }
}