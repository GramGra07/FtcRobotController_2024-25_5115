package org.firstinspires.ftc.teamcode.subsystems.loopTime

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry

@Config
class LoopTimeController(
    timer: ElapsedTime,
    periodics: List<PeriodicLoopTimeObject>,
    spacedObjects: List<SpacedBooleanObject>
) {
    // loopTimeController.spacedObjectOf("name")!!.run(loopTimeController.currentTime)
    private val timer: ElapsedTime
    var loops: Int = 0
    private var lps = 0.0
    var currentTime: Double = 0.0
    private val correctedLPS: Double = 5.0
    private val periodics: List<PeriodicLoopTimeObject>
    private val spacedBooleanObjects: List<SpacedBooleanObject>

    @JvmField
    var loopSaver: Boolean = false

    init {
        this.timer = timer
        this.periodics = periodics
        this.spacedBooleanObjects = spacedObjects
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
        spacedBooleanObjects.forEach { obj ->
            obj.update(currentTime)
        }
    }

    fun telemetry(telemetry: Telemetry) {
        telemetry.addData("LOOP TIME", "")
        telemetry.addData("Timer", "%.1f", currentTime)
        telemetry.addData("Loops", loops)
        telemetry.addData("Current LPS", "%.1f", lps)
    }


    fun spacedObjectOf(name: String): SpacedBooleanObject? {
        return spacedBooleanObjects.find { it.name == name }
    }

    companion object {
        fun LoopTimeController.every(period: Int, func: Runnable) {
            if (this.loops % period == 0) {
                func.run()
            }
        }
    }
}