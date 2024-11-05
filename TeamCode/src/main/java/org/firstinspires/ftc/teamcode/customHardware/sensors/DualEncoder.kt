package org.firstinspires.ftc.teamcode.customHardware.sensors

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

class DualEncoder(
    hw: HardwareMap,
    name1: String,
    name2: String,
    private val subsystemName: String,
    val needsReversing: Boolean = false
) {
    private val encoder: DcMotorEx
    private val reversedEncoder: DcMotorEx

    init {
        encoder = hw.get(DcMotorEx::class.java, name1)
        reversedEncoder = hw.get(DcMotorEx::class.java, name2)
    }

    fun getAverage(): Double {
        return (encoder.getPosition() + (reversedEncoder.getPosition(true))) / 2.0
    }

    fun getMost(): Double {
        return if (encoder.getPosition() > reversedEncoder.getPosition(true)) {
            encoder.getPosition()
        } else {
            reversedEncoder.getPosition(true)
        }
    }

    fun telemetry(telemetry: Telemetry) {
        telemetry.addData("$subsystemName Encoder", "%.1f", encoder.getPosition())
        telemetry.addData(
            "$subsystemName Reversed encoder",
            "%.1f",
            reversedEncoder.getPosition(true)
        )
        telemetry.addData("$subsystemName Average", "%.1f", getMost())
    }

    private fun DcMotorEx.getPosition(reversed: Boolean = false): Double {
        return if (reversed && needsReversing) {
            -this.currentPosition.toDouble()
        } else {
            this.currentPosition.toDouble()
        }
    }
}