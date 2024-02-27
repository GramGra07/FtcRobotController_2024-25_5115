package org.firstinspires.ftc.teamcode.extensions

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit

object MotorExtensions {
    fun initMotor(
        hw: HardwareMap,
        name: String,
        runMode: DcMotor.RunMode,
        direction: DcMotorSimple.Direction? = DcMotorSimple.Direction.FORWARD
    ): DcMotor {
        val t = hw.get(DcMotor::class.java, name)
        t.brake()
        t.direction = direction
        if (runMode == DcMotor.RunMode.RUN_USING_ENCODER) {
            t.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            t.runUsingEncoder()
        } else {
            t.mode = runMode
        }
        return t
    }

    fun DcMotor.brake() {
        this.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    fun DcMotor.coast() {
        this.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
    }

    fun DcMotor.runToPosition() {
        this.mode = DcMotor.RunMode.RUN_TO_POSITION
    }

    fun DcMotor.runUsingEncoder() {
        this.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    fun DcMotor.runWithoutEncoder() {
        this.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }
}