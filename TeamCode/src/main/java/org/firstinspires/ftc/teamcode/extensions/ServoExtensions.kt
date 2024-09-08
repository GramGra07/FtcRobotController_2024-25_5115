package org.firstinspires.ftc.teamcode.extensions

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import kotlin.math.ceil

@Config
object ServoExtensions {
    fun initServo(hw: HardwareMap, name: String): Servo {
        return hw.get(Servo::class.java, name)
    }

    fun Servo.setPose(degrees: Double) {
        val degreeMult = 0.00555555554
        this.position = degreeMult * degrees
    }

    @JvmField
    var servoFlipVal = 62

    @JvmField
    var hcalc = 96.0

    @JvmField
    var lastSetVal: Int = 0

//    fun Servo.calcFlipPose(pose: Double) {
//        val theta = 0
////        val theta: Double = potentiometer.potentAngle()
////        PotentPositions.pastAngleVal = theta
//        val sig = ceil(-0.26 * theta + hcalc) + pose / 2
//        this.setPose(sig)
//        servoFlipVal = sig.toInt()
//        lastSetVal = pose.toInt()
//    }
}