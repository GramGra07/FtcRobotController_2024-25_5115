package org.firstinspires.ftc.teamcode.extensions

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.TouchSensor
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.Voltage.Companion.currentVoltage
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.Voltage.Companion.lowVoltage

object Extensions {
    private val POTENTIOMETER_MAX = 270.0
    private val POTENTIOMETER_MIN = 0.0

    // extend the AnalogInput class to include a method to get the angle of the potentiometer
    fun AnalogInput.potentAngle(): Double {
        return Range.clip(
            (this.voltage - POTENTIOMETER_MIN) / (POTENTIOMETER_MAX - POTENTIOMETER_MIN),
            0.0,
            1.0
        )
    }

    fun DigitalChannel.getPressed(): Boolean = !this.state
    fun DigitalChannel.ledIND(red: DigitalChannel, greenOn: Boolean) {
        this.state = greenOn
        red.state = !greenOn
    }

    fun TouchSensor.getTouchSensor(): Boolean = this.isPressed
}