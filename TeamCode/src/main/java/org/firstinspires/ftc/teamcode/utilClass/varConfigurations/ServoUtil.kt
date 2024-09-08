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
    var openClaw = 100
    fun openClaw(servo: Servo) {
        servo.setPose(openClaw.toDouble())
        lastTimeOpen = timer.seconds()
    }

    @JvmField
    var closeClaw = 200
    fun closeClaw(servo: Servo) {
        servo.setPose(closeClaw.toDouble())
    }
}
