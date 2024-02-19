package org.firstinspires.ftc.teamcode.extensions

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.PastAngle
import org.firstinspires.ftc.teamcode.extensions.Extensions.potentAngle
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.potentiometer
import kotlin.math.ceil

@Config
object ServoExtensions {
    fun Servo.init(hw: HardwareMap, name: String) {
        hw.get(Servo::class.java, name)
    }

    @Override
    fun Servo.setPose(degrees: Double) {
        val degree_mult = 0.00555555554
        lastPosition = degree_mult * degrees
        lastSetVal = degrees.toInt()
        this.position = lastPosition
    }

    var servoFlipVal = 62
    var hcalc = 96.0
    var flipOffset = -30.0
    var lastPosition: Double = 0.0
    var lastSetVal: Int = 0
    fun Servo.calcFlipPose(pose: Double) {
        val theta: Double =
            potentiometer.potentAngle() + flipOffset
        PastAngle.pastAngleVal = theta
        val sig: Double = ceil(-0.26 * theta + hcalc) + pose / 2
        this.position = sig
        servoFlipVal = sig.toInt()
        lastSetVal = pose.toInt()
    }
}