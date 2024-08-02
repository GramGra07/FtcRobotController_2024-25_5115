package org.firstinspires.ftc.teamcode.opModes.testers.OTOS

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles
import java.lang.Math.toDegrees

@TeleOp(group = GroupingTitles.TESTING)
//@Disabled//disabling the opmode
//@Config
class OTOSAngularScalarTuner : LinearOpMode() {
    //declaring the class
    private var deltaAngle = 0.0
    override fun runOpMode() { //if opmode is started
        val robot = HardwareConfig(this, false)
        telemetry.addData("Move the robot 180º","It will tell you the correct angular scalar")
        robot.localizerSubsystem.update(null)
        val startAngle = toDegrees(robot.localizerSubsystem.heading())
        telemetry.addData("startAngle",startAngle)
        telemetry.update()
        val target = startAngle+180.0
        waitForStart()
        while (opModeIsActive()) {
            robot.localizerSubsystem.update(null)
            deltaAngle = startAngle + toDegrees( robot.localizerSubsystem.heading())
            telemetry.addData("deltaAngle",deltaAngle)
            telemetry.addData("target",target)
            telemetry.addData("Angular scalar",target/deltaAngle)
            telemetry.update()
        }
    }
}