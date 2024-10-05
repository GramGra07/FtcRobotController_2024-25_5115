package org.firstinspires.ftc.teamcode.utilClass

import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.setPose
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.ServoUtil

class ServoFunc {
    companion object {
        fun closeClaw(servo: Servo) {
            servo.setPose(ServoUtil.closeClaw)
        }

        fun openClaw(servo: Servo) {
            servo.setPose(ServoUtil.openClaw)
        }
    }
}