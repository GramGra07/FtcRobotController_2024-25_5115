package org.firstinspires.ftc.teamcode.UtilClass

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig.Companion.green1
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig.Companion.green2
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig.Companion.lastTimeOpen
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig.Companion.red1
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig.Companion.red2
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig.Companion.timer
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.ledIND
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.setPose

@Config
object ServoUtil {
    var backClaw = 20

    @JvmField
    var openClaw1 = 100

    @JvmField
    var openClaw2 = 90
    fun openClaw1(servo: Servo) {
        servo.setPose(openClaw1.toDouble())
        green1.ledIND(red1, false)
        lastTimeOpen = timer.seconds()
    }

    fun openClaw2(servo: Servo) {
        green2.ledIND(red2, false)
        servo.setPose(openClaw2.toDouble())
        lastTimeOpen = timer.seconds()
    }

    @JvmField
    var closeClaw1 = 200

    @JvmField
    var closeClaw2 = 10
    fun closeClaw1(servo: Servo) {
        servo.setPose(closeClaw1.toDouble())
        green1.ledIND(red1, true)
    }

    fun closeClaw2(servo: Servo) {
        servo.setPose((closeClaw2.toDouble()))
        green2.ledIND(red2, true)
    }

    @JvmField
    var releaseAirplane = 120

    fun releaseAirplane(servo: Servo) {
        servo.setPose(releaseAirplane.toDouble())
    }

    @JvmField
    var raiseAirplaneVal = 40

    @JvmField
    var airplaneReset = 30

    fun resetAirplane(servo: Servo) {
        servo.setPose(airplaneReset.toDouble())
    }
}
