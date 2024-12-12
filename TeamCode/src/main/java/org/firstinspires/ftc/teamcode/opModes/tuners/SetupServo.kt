package org.firstinspires.ftc.teamcode.opModes.tuners

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.setPose
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles


@TeleOp(group = GroupingTitles.TESTING)
class SetupServo : LinearOpMode() {
    private lateinit var servo: Servo
    override fun runOpMode() {
        servo = hardwareMap.get(Servo::class.java, "servo")
        waitForStart()
        while (opModeIsActive()) {
            telemetry.addData("Press square to set servo to 0.0", "")
            telemetry.update()
            while (!gamepad1.square && opModeIsActive() && !isStopRequested) {
            }
            servo.setPose(0.0)
            telemetry.addData("Press circle to set servo to 180", "")
            telemetry.update()
            while (!gamepad1.circle && opModeIsActive() && !isStopRequested) {
            }
            servo.setPose(180.0)
            telemetry.addData("Press triangle to set servo to 90", "")
            telemetry.update()
            while (!gamepad1.triangle && opModeIsActive() && !isStopRequested) {
            }
            servo.setPose(90.0)

        }
    }
}