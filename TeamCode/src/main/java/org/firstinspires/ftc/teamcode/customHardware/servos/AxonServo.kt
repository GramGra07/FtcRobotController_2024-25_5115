package org.firstinspires.ftc.teamcode.customHardware.servos

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.initServo
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.setPose

class AxonServo(hw: HardwareMap, name: String) {
    val encoder: AnalogInput
    private val servo: Servo
    val name: String

    init {
        this.name = name
        encoder = initAEncoder(hw)
        servo = initServo(hw, name)
    }

    private fun initAEncoder(hw: HardwareMap): AnalogInput {
        return hw.get(AnalogInput::class.java, name + "Encoder")
    }

    fun telemetry(telemetry: Telemetry) {
        telemetry.addData("$name aEncoder", "%.1f", this.getEncoderPosition())
    }

    fun setPosition(degree: Double) {
        servo.setPose(degree)
    }

    fun getEncoderPosition(): Double {
        return (encoder.voltage / 3.3) * 360
    }
}