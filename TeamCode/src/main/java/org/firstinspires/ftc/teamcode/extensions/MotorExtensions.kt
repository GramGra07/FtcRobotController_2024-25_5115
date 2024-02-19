package org.firstinspires.ftc.teamcode.extensions

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

object MotorExtensions {
    fun DcMotor.init(
        hw: HardwareMap,
        name: String,
        runMode: DcMotor.RunMode,
        direction: DcMotorSimple.Direction? = DcMotorSimple.Direction.FORWARD
    ) {
        hw.get(DcMotor::class.java, name)
        this.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        this.direction = direction
        if (runMode == DcMotor.RunMode.RUN_USING_ENCODER) {
            this.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            this.mode = runMode
        } else {
            this.mode = runMode
        }
    }
}