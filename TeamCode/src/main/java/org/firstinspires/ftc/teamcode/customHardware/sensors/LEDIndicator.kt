package org.firstinspires.ftc.teamcode.customHardware.sensors

import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.initDigiChan

class LEDIndicator(hw:HardwareMap,number:Int) {
    private val green:DigitalChannel
    private val red:DigitalChannel
    private val number:Int

    init {
        this.number = number
        green = initDigiChan(hw,"green$number")
        red = initDigiChan(hw,"red$number")
    }
    fun turnGreen(){
        green.state = true
        red.state = false
    }
    fun turnRed(){
        green.state = false
        red.state = true
    }
}