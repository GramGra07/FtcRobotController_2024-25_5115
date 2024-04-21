package org.firstinspires.ftc.teamcode.subsystems.loopTime

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry

class LoopTimeController(timer: ElapsedTime, periodics: List<PeriodicLoopTimeObject>) {
    private val timer: ElapsedTime
    private var loops: Int = 0
    private var lps = 0.0
    private var currentTime: Double = 0.0
    private val correctedLPS: Double = 5.0
    private val periodics: List<PeriodicLoopTimeObject>

    init {
        this.timer = timer
        this.periodics = periodics
    }

    private fun doCalculations() {
        lps = loops / (currentTime - correctedLPS)
        if (currentTime > correctedLPS) {
            loops++
        }
    }

    fun update() {
        currentTime = timer.seconds()
        doCalculations()
        periodics.forEach { obj ->
            obj.rr(currentTime)
            if (obj.check(loops)) {
                obj.run()
            }
        }
    }

    fun telemetry(telemetry: Telemetry) {
        telemetry.addData("Timer", "%.1f", currentTime)
        telemetry.addData("Loops", "%.1f", loops)
        telemetry.addData("Current LPS", "%.1f", lps)
    }
}