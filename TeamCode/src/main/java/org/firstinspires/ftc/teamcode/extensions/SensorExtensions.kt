package org.firstinspires.ftc.teamcode.extensions

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.TouchSensor
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.PastAngle

object SensorExtensions {
    fun initDigiChan(
        hw: HardwareMap,
        name: String,
        mode: DigitalChannel.Mode = DigitalChannel.Mode.OUTPUT
    ): DigitalChannel {
        val t = hw.get(DigitalChannel::class.java, name)
        t.mode = mode
        return t
    }

    fun initPotent(hw: HardwareMap, name: String): AnalogInput {
        val t = hw.get(AnalogInput::class.java, name)
        PastAngle.pastAngleVal = t.potentAngle()
        return t
    }

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

    fun initVSensor(hw: HardwareMap, name: String): VoltageSensor {
        return hw.get(VoltageSensor::class.java, name)
    }

    private fun VoltageSensor.getVoltageCorrected(): Double {
        val minimumVoltage = 11.5
        var result = Double.POSITIVE_INFINITY
        val voltage = this.voltage
        if (voltage > 0) {
            result = Math.min(result, voltage)
        }
        Voltage.lowVoltage = result <= minimumVoltage
        Voltage.currentVoltage = result
        return result
    }

    data class Voltage(var lowVoltage: Boolean, var currentVoltage: Double) {
        companion object {
            var lowVoltage: Boolean = false
            var currentVoltage: Double = 0.0
        }
    }

    fun VoltageSensor.lowVoltage(): Boolean {
        this.getVoltageCorrected()
        return Voltage.lowVoltage
    }

    fun VoltageSensor.currentVoltage(): Double {
        this.getVoltageCorrected()
        return Voltage.currentVoltage
    }
}