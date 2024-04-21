package org.firstinspires.ftc.teamcode.subsystems.loopTime

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig

class LoopTimeController(timer: ElapsedTime,periodics:List<PeriodicLoopTimeObject>) {
    private val timer: ElapsedTime
    private var loops: Int = 0
    private var LPS = 0.0
    private var currentTime: Double = 0.0
    private val correctedLPS: Double = 5.0
    private val periodics: List<PeriodicLoopTimeObject>
    init {
        this.timer = timer
        this.periodics = periodics
    }
    private fun doCalculations() {
        LPS = loops / (currentTime - correctedLPS)
    }
    fun update(){
        currentTime = timer.seconds()
        doCalculations()
        for (obj in periodics){
            obj.rr(currentTime)
            if (obj.check(loops)){
                obj.run()
            }
        }
        if (currentTime > correctedLPS) {
            loops++
        }
    }
    fun telemetry(telemetry: Telemetry){
        telemetry.addData("Timer", "%.1f", currentTime)
        telemetry.addData("Loops", "%.1f", loops)
        telemetry.addData("Current LPS", "%.1f", LPS)
    }
}