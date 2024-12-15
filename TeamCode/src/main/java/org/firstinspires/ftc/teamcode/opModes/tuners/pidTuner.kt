package org.firstinspires.ftc.teamcode.opModes.tuners

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles

@TeleOp(group = GroupingTitles.TESTING) //@Disabled//disabling the opmode

class pidTuner : LinearOpMode() {
    //declaring the class
    override fun runOpMode() { //if opmode is started
        val robot = HardwareConfig(this, false, StartLocation(Alliance.RED))
        waitForStart()
        robot.once() //runs once
        while (opModeIsActive()) { //while the op mode is active
            robot.armSubsystem.setPitchTarget(400.0)
            robot.armSubsystem.setExtendTarget(700.0)
            robot.armSubsystem.update()
            robot.armSubsystem.telemetry(telemetry)
            telemetry.update()
        }
    }
}