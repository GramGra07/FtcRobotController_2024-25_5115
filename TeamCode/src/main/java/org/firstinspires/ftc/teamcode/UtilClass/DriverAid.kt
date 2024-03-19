package org.firstinspires.ftc.teamcode.UtilClass

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import org.firstinspires.ftc.teamcode.rr.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.DriveConfig
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
        drive: MecanumDrive = driveSubsystem.drive!!
    ) {
        val packet = TelemetryPacket()

        if (goToDrone) {
            driveSubsystem.slowModeIsOn = false
            driveSubsystem.isAutoInTeleop = true
            drive.updatePoseEstimate()
            // go to drone scoring position]
        }
        if (turnStraight) {
            driveSubsystem.slowModeIsOn = false
            driveSubsystem.isAutoInTeleop = true
//!            turnAngle = normDelta(Math.toRadians(0.0) - drive.pose.heading.real) TODO test this

//            runningActions.plus(
//                SequentialAction(
//                    drive.actionBuilder(drive.pose).turnTo(Math.toRadians(90.0)).build()
//                )
//            )
        }
        if (turnWing) {
            driveSubsystem.slowModeIsOn = false
            driveSubsystem.isAutoInTeleop = true
//            runningActions.plus(
//                SequentialAction(
//                    drive.actionBuilder(drive.pose).turnTo(Math.toRadians(90.0)).build()
//                )
//            )
        }

        val newActions: MutableList<Action> = ArrayList()
        for (action in runningActions) {
            action.preview(packet.fieldOverlay())
            if (action.run(packet)) {
                newActions.add(action)
            }
        }
        runningActions = newActions
        dash.sendTelemetryPacket(packet)
    }

    fun cMPTurnVect(thetaError: Double): Map<String, Double> {
        val kpTheta = DriveConfig.MOTOR_VELO_PID.p // Proportional gain for theta error

        // Calculate the correction for each motor based on the theta error
        val flPower = -kpTheta * thetaError  // Front-left wheel
        val frPower = kpTheta * thetaError   // Front-right wheel
        val rlPower = -kpTheta * thetaError  // Rear-left wheel
        val rrPower = kpTheta * thetaError   // Rear-right wheel

        // Return a map containing the calculated motor powers
        return mapOf(
            "FL" to flPower,
            "FR" to frPower,
            "RL" to rlPower,
            "RR" to rrPower
        )
    }
}
