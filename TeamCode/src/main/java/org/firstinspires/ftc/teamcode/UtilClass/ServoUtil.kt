package org.firstinspires.ftc.teamcode.UtilClass

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.ledIND
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.setPose
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.clawSubsystem
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.green1
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.green2
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.lastTimeOpen
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.red1
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.red2
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.timer

@Config
object ServoUtil {
    var backClaw = 20

    @JvmField
    var openClaw1 = 100

    @JvmField
    var openClaw2 = 90
    fun openClaw(servo: Servo) {
        if (servo === clawSubsystem.claw1) {
            servo.setPose(openClaw1.toDouble())
            green1.ledIND(red1, false)
        } else if (servo === clawSubsystem.claw2) {
            green2.ledIND(red2, false)
            servo.setPose(openClaw2.toDouble())
        }
        lastTimeOpen = timer.seconds()
    }

    @JvmField
    var closeClaw1 = 200

    @JvmField
    var closeClaw2 = 10
    fun closeClaw(servo: Servo) {
        if (servo == clawSubsystem.claw1) {
            servo.setPose(closeClaw1.toDouble())
            green1.ledIND(red1, true)
        } else if (servo == clawSubsystem.claw2) {
            servo.setPose((closeClaw2.toDouble()))
            green2.ledIND(red2, true)
        }
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
