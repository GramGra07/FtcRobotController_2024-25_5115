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
        ServoPose.position = degree_mult * degrees
        ServoPose.position = degrees
        this.position = ServoPose.position
    }

    var servoFlipVal = 62
    var hcalc = 96.0
    var flipOffset = -30.0
    fun Servo.calcFlipPose(pose: Double) {
        val theta: Double =
            potentiometer.potentAngle() + flipOffset
        PastAngle.pastAngleVal = theta
        val sig: Double = ceil(-0.26 * theta + hcalc) + pose / 2
        this.position = sig
        servoFlipVal = sig.toInt()
        ServoPose.lastSetVal = pose.toInt()
    }

    data class ServoPose(
        val position: Double = 0.0,
        val lastSetVal: Int = 0,
        val servoFlipVal: Int = 62
    ) {
        companion object {
            var position: Double = 0.0
            var lastSetVal = 0
            var servoFlipVal = 62
        }
    }

    fun Servo.getPose(): Double = ServoPose.position
    fun lastSetVal(): Int = ServoPose.lastSetVal
}