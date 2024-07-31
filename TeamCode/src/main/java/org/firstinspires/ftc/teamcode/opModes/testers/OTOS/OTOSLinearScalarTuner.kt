package org.firstinspires.ftc.teamcode.opModes.testers.OTOS

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig
import org.firstinspires.ftc.teamcode.extensions.OTOSExtension.getPose
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles

@TeleOp(group = GroupingTitles.TESTING)
//@Disabled//disabling the opmode
//@Config
class OTOSLinearScalarTuner : LinearOpMode() {
    //declaring the class
    var distance = 30
    private var deltaDistance = 0.0
    override fun runOpMode() { //if opmode is started
        val robot = HardwareConfig(this, false)
        telemetry.addData("Move the robot forward 30 inches","It will tell you the correct linear scalar")
        telemetry.update()
        waitForStart()
        while (opModeIsActive()) {
            deltaDistance = robot.sparkFunOTOS.getPose().y
            telemetry.addData("Linear scalar",distance/deltaDistance)
            telemetry.update()
        }
    }
}