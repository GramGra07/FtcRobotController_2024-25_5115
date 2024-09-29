package org.firstinspires.ftc.teamcode.customHardware.loopTime

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry

@Config
class LoopTimeController(
    // loopTimeController.spacedObjectOf("name")!!.run(loopTimeController.currentTime)
    private val timer: ElapsedTime,
    private val periodics: List<PeriodicLoopTimeObject?>? = null,
    private val spacedBooleanObjects: List<SpacedBooleanObject?>? = null
) {
    var loops: Int = 0
    private var lps = 0.0
    private var currentTime: Double = 0.0
    private val correctedLPS: Double = 5.0
    private var lastSecond:Double = 0.0
    private var lastSecondLPast:Double = 0.0
    private var lastSecondL:Double = 0.0
    private var lastSecondT:Double = 0.0

    @JvmField
    var loopSaver: Boolean = false

    private fun doCalculations() {
        lps = loops / (currentTime - correctedLPS)
        if (currentTime > correctedLPS) {
            loops++
        }
        if (currentTime-1>lastSecondT){
            lastSecondL = loops - lastSecondLPast
            lastSecond = lastSecondL/(currentTime-lastSecondT)
            lastSecondLPast = loops.toDouble()
            lastSecondT = currentTime
        }
    }

    fun update() {
        currentTime = timer.seconds()
        doCalculations()
        periodics?.forEach { obj ->
            obj!!.rr(currentTime)
            if (obj.check(loops)) {
                obj.run()
            }
        }
        spacedBooleanObjects?.forEach { obj ->
            obj!!.update(currentTime)
        }
    }

    fun telemetry(telemetry: Telemetry) {
        telemetry.addData("LOOP TIME", "")
        telemetry.addData("Timer", "%.1f", currentTime)
        telemetry.addData("Loops", loops)
        telemetry.addData("Current LPS", "%.1f", lps)
//        telemetry.addData("lastsecondloops",lastSecondL)
//        telemetry.addData("pastlastsecondloops",lastSecondLPast)
//        telemetry.addData("lastsecondt",lastSecondT)
        telemetry.addData("Last Second LPS","%.1f",lastSecond)
    }


    fun spacedObjectOf(name: String): SpacedBooleanObject? {
        return spacedBooleanObjects?.find { it!!.name == name }
    }

    companion object {
        fun LoopTimeController.every(period: Int, func: Runnable) {
            if (this.loops % period == 0) {
                func.run()
            }
        }
    }
}