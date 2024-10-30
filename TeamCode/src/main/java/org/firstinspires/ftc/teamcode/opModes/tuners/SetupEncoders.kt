package org.firstinspires.ftc.teamcode.opModes.tuners

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx


@TeleOp
class SetupEncoders : LinearOpMode() {
    lateinit var motorEx: List<DcMotorEx>
    override fun runOpMode() {
        motorEx = listOf(
            hardwareMap.get(DcMotorEx::class.java, "pitchMotor"),
            hardwareMap.get(DcMotorEx::class.java, "pitchMotor2"),
            hardwareMap.get(DcMotorEx::class.java, "extendMotor"),
            hardwareMap.get(DcMotorEx::class.java, "extendMotor2")
        )
        waitForStart()
        if (opModeIsActive()) {
            motorEx.forEach {
                it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                it.mode = DcMotor.RunMode.RUN_USING_ENCODER
            }
        }
    }
}