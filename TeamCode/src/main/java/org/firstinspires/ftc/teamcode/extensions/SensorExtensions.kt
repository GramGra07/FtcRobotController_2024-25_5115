package org.firstinspires.ftc.teamcode.extensions

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.UtilClass.varConfigurations.PotentPositions
import kotlin.math.min

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
        PotentPositions.pastAngleVal = t.potentAngle()
        return t
    }


    // extend the AnalogInput class to include a method to get the angle of the potentiometer
    fun AnalogInput.potentAngle(): Double {
        val potentMax = 270.0
        return Range.clip(
            potentMax / 3.3 * this.voltage,
            0.0,
            1.0
        )
    }

    fun DigitalChannel.getPressed(): Boolean = !this.state

    fun initVSensor(hw: HardwareMap, name: String): VoltageSensor {
        return hw.get(VoltageSensor::class.java, name)
    }

    private fun VoltageSensor.getVoltageCorrected(): Double {
        val minimumVoltage = 11.5
        var result = Double.POSITIVE_INFINITY
        val voltage = this.voltage
        if (voltage > 0) {
            result = min(result, voltage)
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

    fun VoltageSensor.telemetry(telemetry: Telemetry) {
        if (this.lowVoltage()) {
            telemetry.addData("", "We have a low battery:")
        }
        telemetry.addData(
            "Voltage",
            "%.1f",
            this.currentVoltage()
        )
    }

    fun AnalogInput.telemetryPotent(telemetry: Telemetry) {
        telemetry.addData(
            "Potentiometer",
            "%.2f",
            this.potentAngle()
        )
    }
}