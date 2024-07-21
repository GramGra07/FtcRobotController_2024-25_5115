package org.firstinspires.ftc.teamcode.utilClass

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.roadrunner.Action
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem


object DriverAid {
    var turnAngle = 0.0
    private val dash = FtcDashboard.getInstance()
    private var runningActions: List<Action> = ArrayList<Action>()
    fun doDriverAid(
        driveSubsystem: DriveSubsystem,
        goToDrone: Boolean,
        turnStraight: Boolean,
        turnWing: Boolean,
        cancel: Boolean,
    ) {

    }
}
