package org.firstinspires.ftc.teamcode.subsystems.loopTime

import org.firstinspires.ftc.robotcore.external.Telemetry

class PeriodicLoopTimeObject(name:String,everyLoopNum:Int,func:Runnable) {
    private val func:Runnable
    val name:String
    private val interval:Int
    private var refreshRate: Double = 0.0
    private var pastRefreshRate: Double = refreshRate
    private var realRefreshRate: Double = 0.0
    private var pastTimeRR: Double = 0.0

    fun rr(currentTime:Double){
        if (refreshRate != pastRefreshRate) {
            realRefreshRate = currentTime - pastTimeRR
            pastRefreshRate = refreshRate
            pastTimeRR = currentTime
        }
    }
    fun run(){
        func
    }
    fun check(loops:Int):Boolean{
        return loops%interval == 0
    }
    init {
        this.name = name
        this.interval = everyLoopNum
        this.func=func
    }
    fun telemetry(telemetry: Telemetry){
        telemetry.addData("$name Refresh Rate", "%.1f", realRefreshRate)
    }
}