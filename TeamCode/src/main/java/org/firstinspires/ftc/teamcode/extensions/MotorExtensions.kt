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
    ): DcMotorEx {
        val t = hw.get(DcMotorEx::class.java, name)
        t.brake()
        t.direction = direction
        if (runMode == DcMotor.RunMode.RUN_USING_ENCODER) {
//            t.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            t.runUsingEncoder()
        } else {
            t.mode = runMode
        }
        return t
    }

    fun DcMotorEx.brake() {
        this.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    fun DcMotorEx.coast() {
        this.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
    }

    fun DcMotorEx.runToPosition() {
        this.mode = DcMotor.RunMode.RUN_TO_POSITION
    }

    fun DcMotorEx.runUsingEncoder() {
        this.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    fun DcMotorEx.runWithoutEncoder() {
        this.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun DcMotorEx.getMotorCurrent(): Double {
        return this.getCurrent(CurrentUnit.AMPS)
    }
}