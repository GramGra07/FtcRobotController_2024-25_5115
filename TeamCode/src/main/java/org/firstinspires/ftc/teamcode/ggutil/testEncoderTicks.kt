package org.firstinspires.ftc.teamcode.ggutil

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor

@TeleOp
@Disabled
class testEncoderTicks : LinearOpMode() {
    private lateinit var enc1: DcMotor
    override fun runOpMode() { //if opmode is started
        enc1 = hardwareMap.get(DcMotor::class.java, "enc1")
        enc1.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        enc1.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        waitForStart()
        while (opModeIsActive()) { //while the op mode is active
            telemetry.addData("enc1", enc1.currentPosition)
            telemetry.update()
        }
    }
}