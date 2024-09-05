package org.firstinspires.ftc.teamcode.utilClass

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.roadrunner.Action


object DriverAid {
    var turnAngle = 0.0
    private val dash = FtcDashboard.getInstance()
    private var runningActions: List<Action> = ArrayList<Action>()
    fun doDriverAid() {

    }
}
