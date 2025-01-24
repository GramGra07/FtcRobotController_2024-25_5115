package org.firstinspires.ftc.teamcode.opModes.tuners

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance

//@TeleOp(group = GroupingTitles.tele) //@Disabled//disabling the opmode

class startupTest(val alliance: Alliance) : LinearOpMode() {
    //declaring the class
    override fun runOpMode() { //if opmode is started
        val robot =
            HardwareConfig(this, true, StartLocation(this.alliance)) //initializing the robot
        waitForStart()
        robot.once() //runs once
        while (opModeIsActive()) { //while the op mode is active
            robot.localizerSubsystem.drive.updatePoseEstimate()
            telemetry.addData("x", robot.localizerSubsystem.drive.pose.position.x)
            telemetry.addData("y", robot.localizerSubsystem.drive.pose.position.y)
            telemetry.addData("heading", robot.localizerSubsystem.drive.pose.heading)
            robot.scoringSubsystem.setup()
            telemetry.addData("arm", robot.armSubsystem.pitchEncoder.currentPosition)
            telemetry.addData("extend", robot.armSubsystem.extendEncoder.getMost())
            telemetry.update()
        }
    }
}