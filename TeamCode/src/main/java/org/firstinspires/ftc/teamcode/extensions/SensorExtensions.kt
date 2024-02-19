package org.firstinspires.ftc.teamcode.extensions

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.PastAngle
import org.firstinspires.ftc.teamcode.extensions.Extensions.potentAngle

object SensorExtensions {
    fun DigitalChannel.init(
        hw: HardwareMap,
        name: String,
        mode: DigitalChannel.Mode = DigitalChannel.Mode.OUTPUT
    ) {
        this.mode = mode
        hw.get(DigitalChannel::class.java, name)
    }

    fun AnalogInput.init(hw: HardwareMap, name: String) {
        hw.get(AnalogInput::class.java, name)
        PastAngle.pastAngleVal = this.potentAngle()
    }

    fun VoltageSensor.init(hw: HardwareMap, name: String) {
        hw.get(VoltageSensor::class.java, name)
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