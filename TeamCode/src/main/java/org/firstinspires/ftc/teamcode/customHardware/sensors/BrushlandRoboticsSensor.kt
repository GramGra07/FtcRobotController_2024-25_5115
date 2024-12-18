package org.firstinspires.ftc.teamcode.customHardware.sensors

import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

class BrushlandRoboticsSensor(hw: HardwareMap, val name: String) {
    var pin1: DigitalChannel
    var pin2: DigitalChannel

    init {
        pin1 = hw.get(DigitalChannel::class.java, "${name}1")
        pin2 = hw.get(DigitalChannel::class.java, "${name}2")
    }

    enum class Color {
        RED, BLUE, YELLOW, NONE
    }

    var color = Color.NONE

    fun telemetry(telemetry: Telemetry) {
//        telemetry.addData("Sensor 1:",pin1.state)
//        telemetry.addData("Sensor 2:",pin2.state)
        telemetry.addData("$name Color", getDigiColor())
    }

    fun getDigiColor(): Color {
        return if (pin1.state && pin2.state) {
            Color.YELLOW
        } else if (!pin1.state && pin2.state) {
            Color.RED
        } else if (pin1.state && !pin2.state) {
            Color.BLUE
        } else {
            Color.NONE
        }
    }
}