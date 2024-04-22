package org.firstinspires.ftc.teamcode.subsystems.loopTime

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig

class SpacedBooleanObject (name:String,spacing:Double, function:Runnable) { // purpose is to have things that can only happen every so often
    private var lastTime: Double = 0.0
    private var spacing: Double
    private var canRun: Boolean = false
    val name: String
    private val runnable: Runnable

    init {
        this.spacing = spacing
        canRun = true
        this.name = name
        this.runnable = function
    }
    fun update(currentTime: Double) {
        if (currentTime - lastTime > spacing) {
            canRun = true
            lastTime = currentTime
        }
    }
    fun telemetry(telemetry: Telemetry) {
        telemetry.addData("$name Can Run", canRun)
    }
    fun run(currentTime:Double){
        if (canRun){
            runnable.run()
        }
        canRun = false
        lastTime = currentTime
    }
}