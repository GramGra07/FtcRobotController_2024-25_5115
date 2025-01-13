package org.firstinspires.ftc.teamcode.extensions

import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.customHardware.sensors.BrushlandRoboticsSensor

object BlinkExtensions {
    fun initLights(
        hw: HardwareMap,
        name: String,
        pattern: RevBlinkinLedDriver.BlinkinPattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK
    ): RevBlinkinLedDriver {
        val t = hw.get(RevBlinkinLedDriver::class.java, name)
        t.setPattern(pattern)
        LEDColor.currentColor = pattern.toString()
        return t
    }

    fun RevBlinkinLedDriver.setPatternCo(pattern: RevBlinkinLedDriver.BlinkinPattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK) {
        this.setPattern(pattern)
        LEDColor.currentColor = pattern.toString()
    }

    fun BrushlandRoboticsSensor.Color.blinkFrom(): RevBlinkinLedDriver.BlinkinPattern {
        return if (this == BrushlandRoboticsSensor.Color.RED) {
            RevBlinkinLedDriver.BlinkinPattern.RED
        } else if (this == BrushlandRoboticsSensor.Color.BLUE) {
            RevBlinkinLedDriver.BlinkinPattern.BLUE
        } else if (this == BrushlandRoboticsSensor.Color.YELLOW) {
            RevBlinkinLedDriver.BlinkinPattern.YELLOW
        } else {
            RevBlinkinLedDriver.BlinkinPattern.HOT_PINK
        }
    }

    private data class LEDColor(val currentColor: String) {
        companion object {
            var currentColor: String = "HOT_PINK"
        }
    }

    fun RevBlinkinLedDriver.currentColor(): String = LEDColor.currentColor

}