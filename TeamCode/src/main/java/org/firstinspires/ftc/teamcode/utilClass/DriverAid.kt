package org.firstinspires.ftc.teamcode.utilClass

import CancelableFollowTrajectoryAction
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import org.firstinspires.ftc.teamcode.rr.MecanumDrive
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
}
