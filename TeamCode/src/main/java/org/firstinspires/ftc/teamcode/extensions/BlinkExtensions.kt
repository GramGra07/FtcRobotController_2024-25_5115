package org.firstinspires.ftc.teamcode.extensions

import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.hardware.HardwareMap

object BlinkExtensions {
    fun RevBlinkinLedDriver.init(
        hw: HardwareMap,
        name: String,
        pattern: RevBlinkinLedDriver.BlinkinPattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK
    ) {
        hw.get(RevBlinkinLedDriver::class.java, name)
        this.setPattern(pattern)
        LEDColor.currentColor = pattern.toString()
    }

    fun RevBlinkinLedDriver.setPatternCo(pattern: RevBlinkinLedDriver.BlinkinPattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK) {
        this.setPattern(pattern)
        LEDColor.currentColor = pattern.toString()
    }

    private data class LEDColor(val currentColor: String) {
        companion object {
            var currentColor: String = "HOT_PINK"
        }
    }

    fun RevBlinkinLedDriver.currentColor(): String = LEDColor.currentColor

}