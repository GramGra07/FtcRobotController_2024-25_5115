package org.firstinspires.ftc.teamcode.UtilClass

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.extensions.Extensions.ledIND
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.setPose
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.green1
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.green2
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.lastTimeOpen
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.red1
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.red2
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.timer

@Config
object ServoUtil {
    var backClaw = 20
    var openClaw1 = 100
    var openClaw2 = 90
    fun openClaw(servo: Servo) {
        if (servo === HardwareConfig.claw1) {
            servo.setPose(openClaw1.toDouble())
            green1.ledIND(red1, false)
        } else if (servo === HardwareConfig.claw2) {
            green2.ledIND(red2, false)
            servo.setPose(openClaw2.toDouble())
        }
        lastTimeOpen = timer.seconds()
    }

    var closeClaw1 = 200
    var closeClaw2 = 10
    fun closeClaw(servo: Servo?) {
        if (servo == HardwareConfig.claw1) {
            servo.setPose(closeClaw1.toDouble())
            green1.ledIND(red1, true)
        } else if (servo == HardwareConfig.claw2) {
            servo.setPose((closeClaw2.toDouble()))
            green2.ledIND(red2, true)
        }
    }

    var releaseAirplane = 120

    fun releaseAirplane(servo: Servo) {
        servo.setPose(releaseAirplane.toDouble())
    }

    var raiseAirplaneVal = 40
    var airplaneReset = 30

    fun resetAirplane(servo: Servo) {
        servo.setPose(airplaneReset.toDouble())
    }
}
