package org.firstinspires.ftc.teamcode.customHardware.servos

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.setPose

class SynchronizedServo(var hw: HardwareMap, var name: String, private var axonServo: Boolean) {
    private lateinit var servo1: Servo
    private lateinit var servo2: Servo

    private lateinit var servo1Axon: AxonServo
    private lateinit var servo2Axon: AxonServo

    init {
        when (axonServo) {
            true -> {
                servo1Axon = AxonServo(hw, name + '1')
                servo2Axon = AxonServo(hw, name + '2')
            }

            false -> {
                servo1 = hw.get(Servo::class.java, name)
                servo2 = hw.get(Servo::class.java, name)
            }
        }
    }

    fun setPose(pose: Double) {
        when (axonServo) {
            true -> {
                servo1Axon.setPosition(90 + pose)
                servo2Axon.setPosition(90 - pose)
            }

            false -> {
                servo1.setPose(pose)
                servo2.setPose(pose)
            }
        }
    }

}