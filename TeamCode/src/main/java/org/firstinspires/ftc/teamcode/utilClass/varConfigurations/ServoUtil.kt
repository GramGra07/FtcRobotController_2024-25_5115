package org.firstinspires.ftc.teamcode.utilClass.varConfigurations

//import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig.Companion.green1
//import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig.Companion.green2
//import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig.Companion.red1
//import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig.Companion.red2
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig.Companion.lastTimeOpen
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig.Companion.timer
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.setPose

@Config
object ServoUtil {
    @JvmField
    var backClaw = 20

    @JvmField
    var openClaw1 = 100

    @JvmField
    var openClaw2 = 90
    fun openClaw1(servo: Servo) {
        servo.setPose(openClaw1.toDouble())
        lastTimeOpen = timer.seconds()
    }

    fun openClaw2(servo: Servo) {
        servo.setPose(openClaw2.toDouble())
        lastTimeOpen = timer.seconds()
    }

    @JvmField
    var closeClaw1 = 200

    @JvmField
    var closeClaw2 = 10
    fun closeClaw1(servo: Servo) {
        servo.setPose(closeClaw1.toDouble())
    }

    fun closeClaw2(servo: Servo) {
        servo.setPose((closeClaw2.toDouble()))
    }
}
