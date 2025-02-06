package org.firstinspires.ftc.teamcode.opModes.tuners

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles

@TeleOp(group = GroupingTitles.tele) //@Disabled//disabling the opmode

//@Disabled
class startupTest() : LinearOpMode() {
    //declaring the class
    lateinit var motorEx: List<DcMotorEx>
    override fun runOpMode() { //if opmode is started
        val robot =
            HardwareConfig(this, true, StartLocation(Alliance.RED)) //initializing the robot
        motorEx = listOf(
            hardwareMap.get(DcMotorEx::class.java, "motorFrontRight"),
            hardwareMap.get(DcMotorEx::class.java, "extendMotor"),
            hardwareMap.get(DcMotorEx::class.java, "extendMotor2")
        )
        waitForStart()
        robot.once() //runs once
        motorEx.forEach {
            it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            it.mode = DcMotor.RunMode.RUN_USING_ENCODER
        }
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