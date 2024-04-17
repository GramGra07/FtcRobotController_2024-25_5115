package org.firstinspires.ftc.teamcode.customHardware.sensors

import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

class BeamBreakSensor(hw: HardwareMap,name: String) {
    val name: String
    private val beamBreak: DigitalChannel
    init {
        this.name = name
        beamBreak = initBeamBreak(hw)
    }
    private fun initBeamBreak(hw: HardwareMap): DigitalChannel {
        return hw.get(DigitalChannel::class.java, name)
    }
    private fun getPressed(): Boolean {
        return !this.beamBreak.state
    }
    fun isBroken(): Boolean = this.getPressed()
    private fun isOpen(): Boolean = !this.getPressed()
    fun telemetry(telemetry: Telemetry){
        telemetry.addData(name,isBroken())
        telemetry.addData("is open",isOpen())
    }
}