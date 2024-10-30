package org.firstinspires.ftc.teamcode.customHardware.servos

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.setPose

class SynchronizedServo(var hw: HardwareMap, var name: String) {
    private var servo1: Servo
    private var servo2: Servo

    init {
        servo1 = hw.get(Servo::class.java, "${name}1")
        servo2 = hw.get(Servo::class.java, "${name}2")
    }

    fun setPose(pose: Double) {
        servo1.setPose(pose)
        servo2.setPose(pose)
    }

}