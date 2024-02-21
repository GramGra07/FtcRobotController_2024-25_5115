package org.firstinspires.ftc.teamcode.opModes.autoSoftware

import org.firstinspires.ftc.teamcode.hummingbird.Flower
import org.firstinspires.ftc.teamcode.Enums.Alliance
import org.firstinspires.ftc.teamcode.Enums.EndPose
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.EndPoseVals
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.StartPose
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.Companion.updatePose
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive

object endPose {
    private var endPoseRightRed = Flower(50.0, -EndPoseVals.outside.toDouble(), Math.toRadians(0.0))
    private var endPoseLeftRed = Flower(50.0, -EndPoseVals.inside.toDouble(), Math.toRadians(0.0))
    private var endPoseRightBlue = Flower(50.0, EndPoseVals.inside.toDouble(), Math.toRadians(0.0))
    private var endPoseLeftBlue = Flower(50.0, EndPoseVals.outside.toDouble(), Math.toRadians(0.0))

    // returns a trajectory sequence to go to the end pose
    fun goToEndPose(endPose: EndPose, drive: MecanumDrive) {
        var pose: Flower? = drive.poseEstimate
        when (endPose) {
            EndPose.StartingPosition -> {
                pose = autoHardware.START_POSE
                drive.followTrajectorySequenceAsync(
                    drive.trajectorySequenceBuilder(drive.poseEstimate)
                        .lineToLinearHeading(pose)
                        .build()
                )
            }

            EndPose.LEFT -> {
                when (StartPose.alliance) {
                    Alliance.RED -> pose = endPoseLeftRed
                    Alliance.BLUE -> pose = endPoseLeftBlue
                }
                drive.followTrajectorySequenceAsync(
                    drive.trajectorySequenceBuilder(drive.poseEstimate)
                        .lineToLinearHeading(pose)
                        .build()
                )
            }

            EndPose.RIGHT -> {
                when (StartPose.alliance) {
                    Alliance.RED -> pose = endPoseRightRed
                    Alliance.BLUE -> pose = endPoseRightBlue
                }
                drive.followTrajectorySequenceAsync(
                    drive.trajectorySequenceBuilder(drive.poseEstimate)
                        .lineToLinearHeading(pose)
                        .build()
                )
            }

            else -> {}
        }
        updatePose(drive)
    }
}
