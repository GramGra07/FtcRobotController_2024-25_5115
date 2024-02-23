package org.firstinspires.ftc.teamcode.Trajectories

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.Enums.Alliance
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.StartPose
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive
import org.firstinspires.ftc.teamcode.opModes.rr.trajectorysequence.TrajectorySequence

object spikeNavTraj {
    fun midPiNav(drive: MecanumDrive): TrajectorySequence {
        return if (StartPose.alliance == Alliance.RED) {
            drive.trajectorySequenceBuilder(drive.poseEstimate)
                .splineToLinearHeading(
                    Pose2d(
                        autoHardware.START_POSE.x + 8,
                        autoHardware.START_POSE.y + 25,
                        autoHardware.START_POSE.heading
                    ), autoHardware.START_POSE.heading
                )
                .build()
        } else {
            drive.trajectorySequenceBuilder(drive.poseEstimate)
                .splineToLinearHeading(
                    Pose2d(
                        autoHardware.START_POSE.x + 8,
                        autoHardware.START_POSE.y - 25,
                        autoHardware.START_POSE.heading
                    ), autoHardware.START_POSE.heading
                )
                .build()
        }
    }

    var rotate = 70
    fun fwdLeft(drive: MecanumDrive): TrajectorySequence {
        return if (StartPose.alliance == Alliance.BLUE) {
            drive.trajectorySequenceBuilder(drive.poseEstimate)
                .splineToLinearHeading(
                    Pose2d(
                        autoHardware.START_POSE.x,
                        autoHardware.START_POSE.y - 23,
                        autoHardware.START_POSE.heading + rotate - Math.toRadians(
                            10.0
                        )
                    ),
                    autoHardware.START_POSE.heading + rotate - Math.toRadians(
                        10.0
                    )
                )
                .build()
        } else {
            drive.trajectorySequenceBuilder(drive.poseEstimate)
                .splineToLinearHeading(
                    Pose2d(
                        autoHardware.START_POSE.x,
                        autoHardware.START_POSE.y + 22,
                        autoHardware.START_POSE.heading + rotate - Math.toRadians(
                            10.0
                        )
                    ),
                    autoHardware.START_POSE.heading + rotate - Math.toRadians(
                        10.0
                    )
                )
                .build()
        }
    }

    fun fwdRight(drive: MecanumDrive): TrajectorySequence {
        return if (StartPose.alliance == Alliance.RED) {
            drive.trajectorySequenceBuilder(drive.poseEstimate)
                .splineToLinearHeading(
                    Pose2d(
                        autoHardware.START_POSE.x,
                        autoHardware.START_POSE.y + 23,
                        autoHardware.START_POSE.heading - rotate
                    ), autoHardware.START_POSE.heading - rotate
                )
                .build()
        } else {
            drive.trajectorySequenceBuilder(drive.poseEstimate)
                .splineToLinearHeading(
                    Pose2d(
                        autoHardware.START_POSE.x,
                        autoHardware.START_POSE.y - 23,
                        autoHardware.START_POSE.heading - rotate
                    ), autoHardware.START_POSE.heading - rotate
                )
                .build()
        }
    }
}
