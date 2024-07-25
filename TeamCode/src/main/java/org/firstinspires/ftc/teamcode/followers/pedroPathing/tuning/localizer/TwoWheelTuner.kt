package org.firstinspires.ftc.teamcode.followers.pedroPathing.tuning.localizer

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles

@TeleOp(group = GroupingTitles.pedroTuning) //@Disabled//disabling the opmode

class TwoWheelTuner : LinearOpMode() {
    //declaring the class
    override fun runOpMode() { //if opmode is started
        val robot = HardwareConfig(this, false)
        waitForStart()
        while (opModeIsActive()) { //while the op mode is active
            robot.driveSubsystem.setArtificialPower(0.0, 1.0)
            robot.doBulk()
        }
    }
}