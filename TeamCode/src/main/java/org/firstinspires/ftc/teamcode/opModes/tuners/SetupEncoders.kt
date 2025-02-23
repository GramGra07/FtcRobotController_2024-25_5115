package org.firstinspires.ftc.teamcode.opModes.tuners

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles


@TeleOp(group = GroupingTitles.TESTING)
@Disabled
class SetupEncoders : LinearOpMode() {
    lateinit var motorEx: List<DcMotorEx>
    override fun runOpMode() {
        motorEx = listOf(
            hardwareMap.get(DcMotorEx::class.java, "motorFrontRight"),
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