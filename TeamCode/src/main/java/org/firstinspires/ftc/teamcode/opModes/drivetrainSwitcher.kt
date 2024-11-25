package org.firstinspires.ftc.teamcode.opModes

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles
import org.firstinspires.ftc.teamcode.utilClass.drivetrain.Drivetrain.Companion.drivetrains
import org.firstinspires.ftc.teamcode.utilClass.storage.CurrentDrivetrain

@TeleOp(group = GroupingTitles.TESTING)
@Disabled
class drivetrainSwitcher : LinearOpMode() {
    override fun runOpMode() {
        var index = drivetrains.indexOfFirst { it.name == CurrentDrivetrain.currentDrivetrain.name }
        var rbHeld = false
        var lbHeld = false
        while (opModeInInit()) {
            telemetry.addData("Current Drivetrain", CurrentDrivetrain.currentDrivetrain.name)
            telemetry.addData("Current Drivetrain Type", CurrentDrivetrain.currentDrivetrain.type)
            telemetry.update()
            if (gamepad1.right_bumper && !rbHeld) {
                index++
                if (index >= drivetrains.size) {
                    index = 0
                }
                CurrentDrivetrain.currentDrivetrain = drivetrains[index]
            }
            if (gamepad1.left_bumper && !lbHeld) {
                index--
                if (index < 0) {
                    index = drivetrains.size - 1
                }
                CurrentDrivetrain.currentDrivetrain = drivetrains[index]
            }
            rbHeld = gamepad1.right_bumper
            lbHeld = gamepad1.left_bumper
            if (opModeIsActive()) break
        }
//        waitForStart()
        while (opModeIsActive()) {
            telemetry.addData("Current Drivetrain", CurrentDrivetrain.currentDrivetrain.name)
            telemetry.addData("Current Drivetrain Type", CurrentDrivetrain.currentDrivetrain.type)
            telemetry.update()
            if (gamepad1.right_bumper && !rbHeld) {
                index++
                if (index >= drivetrains.size) {
                    index = 0
                }
                CurrentDrivetrain.currentDrivetrain = drivetrains[index]
            }
            if (gamepad1.left_bumper && !lbHeld) {
                index--
                if (index < 0) {
                    index = drivetrains.size - 1
                }
                CurrentDrivetrain.currentDrivetrain = drivetrains[index]
            }
            rbHeld = gamepad1.right_bumper
            lbHeld = gamepad1.left_bumper
        }
    }

}