package org.firstinspires.ftc.teamcode.customHardware.servos

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.getEncoderPosition
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.initServo

class AxonServo(hw: HardwareMap, name: String) {
    val encoder: AnalogInput
    val servo: Servo
    val name: String
    var position: Double? = null

    init {
        this.name = name
        encoder = initAEncoder(hw)
        servo = initServo(hw, name)
        position = this.getEncoderPosition()
    }

    private fun initAEncoder(hw: HardwareMap): AnalogInput {
        return hw.get(AnalogInput::class.java, name + "Encoder")
    }
    fun telemetry(telemetry: Telemetry){
        if (position == null){
            this.getEncoderPosition()
        }
        telemetry.addData(name,position)
    }
}