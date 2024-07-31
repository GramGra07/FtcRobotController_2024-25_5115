package org.firstinspires.ftc.teamcode.opModes.testers.OTOS

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig
import org.firstinspires.ftc.teamcode.extensions.OTOSExtension.getPose
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles

@TeleOp(group = GroupingTitles.TESTING)
//@Disabled//disabling the opmode
//@Config
class OTOSAngularScalarTuner : LinearOpMode() {
    //declaring the class
    var angle = 180
    private var deltaAngle = 0.0
    override fun runOpMode() { //if opmode is started
        val robot = HardwareConfig(this, false)
        telemetry.addData("Move the robot forward 180º","It will tell you the correct angular scalar")
        telemetry.update()
        waitForStart()
        while (opModeIsActive()) {
            deltaAngle = robot.sparkFunOTOS.getPose().h
            telemetry.addData("Linear scalar",angle/deltaAngle)
            telemetry.update()
        }
    }
}