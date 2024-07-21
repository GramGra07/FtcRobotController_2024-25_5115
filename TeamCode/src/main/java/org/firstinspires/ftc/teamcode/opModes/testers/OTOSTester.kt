package org.firstinspires.ftc.teamcode.opModes.testers

import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.extensions.OTOSExtension.initOTOS
import org.firstinspires.ftc.teamcode.extensions.OTOSExtension.telemetry

@TeleOp(group = "a")
//@Disabled//disabling the opmode

class OTOSTester : LinearOpMode() {
    //declaring the class
    override fun runOpMode() { //if opmode is started
        val sensor = initOTOS(this.hardwareMap, "spark", Pose2D(0.0, 0.0, 0.0))
        waitForStart()

        while (opModeIsActive()) {
            sensor.telemetry(telemetry)
        }
    }
}