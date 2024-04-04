package org.firstinspires.ftc.teamcode.UtilClass

import CancelableFollowTrajectoryAction
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
        cancel: Boolean,
        drive: MecanumDrive = driveSubsystem.drive
    ) {
        val packet = TelemetryPacket()

        val cancelableFollowing = CancelableFollowTrajectoryAction(
            drive.actionBuilder(drive.pose)
//            .splineToConstantHeading(Vector2d(x = 0.0,y=0.0),0.0)
                .build()
        )
        if (goToDrone) {
            driveSubsystem.slowModeIsOn = false
            driveSubsystem.isAutoInTeleop = true
//            drive.updatePoseEstimate()
            cancelableFollowing.run(packet)
            // go to drone scoring position]
        }
        if (turnStraight) {
            driveSubsystem.slowModeIsOn = false
            driveSubsystem.isAutoInTeleop = true
//!            turnAngle = normDelta(Math.toRadians(0.0) - drive.pose.heading.real)
        }
        if (turnWing) {
            driveSubsystem.slowModeIsOn = false
            driveSubsystem.isAutoInTeleop = true
        }
        if (cancel) {
            driveSubsystem.slowModeIsOn = false
            driveSubsystem.isAutoInTeleop = false
            cancelableFollowing.cancelAbruptly()
        }
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
