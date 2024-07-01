package org.firstinspires.ftc.teamcode.actions.teleop

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import org.firstinspires.ftc.teamcode.followers.rr.MecanumDrive


class CancelableFollowTrajectoryAction(
    private var action: Action,
    var drive: MecanumDrive,
    private var cancelled: Boolean = false
) :
    Action {

    override fun run(p: TelemetryPacket): Boolean {
        if (cancelled) {
            drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
            return false
        }
        return action.run(p)
    }

    fun cancelAbruptly() {
        cancelled = true
    }
}
