package org.firstinspires.ftc.teamcode.customHardware.autoHardware

import com.acmerobotics.roadrunner.Pose2d
import org.firstinspires.ftc.teamcode.startEnums.Alliance
import org.firstinspires.ftc.teamcode.startEnums.StartSide
import org.firstinspires.ftc.teamcode.storage.PoseStorage

data class StartPose(
    var startLocation: StartLocation
) {
    var pose: Pose2d

    init {
        this.pose = getStartPose(startLocation)
        PoseStorage.currentPose = pose
    }

    private fun getStartPose(startLocation: StartLocation): Pose2d {
        val spot = when (startLocation.alliance) {
            Alliance.BLUE -> {
                when (startLocation.startSide) {
                    StartSide.LEFT -> Pose2d(12.0, -63.0, Math.toRadians(90.0))
                    StartSide.RIGHT -> Pose2d(12.0, -63.0, Math.toRadians(90.0))
                }
            }

            Alliance.RED -> {
                when (startLocation.startSide) {
                    StartSide.LEFT -> Pose2d(-12.0, -63.0, Math.toRadians(90.0))
                    StartSide.RIGHT -> Pose2d(-12.0, -63.0, Math.toRadians(90.0))
                }
            }
        }
        PoseStorage.currentPose = spot
        return spot
    }
}
