package org.firstinspires.ftc.teamcode.opModes.autoSoftware

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.Enums.Alliance
import org.firstinspires.ftc.teamcode.Enums.EndPose
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.EndPoseVals.inside
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.EndPoseVals.outside
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.StartPose
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive

object endPose {
    var endPoseRightRed = Pose2d(50.0, -outside.toDouble(), Math.toRadians(0.0))
    var endPoseLeftRed = Pose2d(50.0, -inside.toDouble(), Math.toRadians(0.0))
    var endPoseRightBlue = Pose2d(50.0, inside.toDouble(), Math.toRadians(0.0))
    var endPoseLeftBlue = Pose2d(50.0, outside.toDouble(), Math.toRadians(0.0))

    // returns a trajectory sequence to go to the end pose
    @JvmStatic
    fun goToEndPose(endPose: EndPose, drive: MecanumDrive) {
        var pose = drive.poseEstimate
        when (endPose) {
            EndPose.StartingPosition -> {
                pose = AutoHardware.START_POSE
                drive.followTrajectorySequenceAsync(
                    drive.trajectorySequenceBuilder(drive.poseEstimate)
                        .lineToLinearHeading(pose)
                        .build()
                )
            }

            EndPose.LEFT -> {
                pose = when (StartPose.alliance) {
                    Alliance.RED -> endPoseLeftRed
                    Alliance.BLUE -> endPoseLeftBlue
                }
                drive.followTrajectorySequenceAsync(
                    drive.trajectorySequenceBuilder(drive.poseEstimate)
                        .lineToLinearHeading(pose)
                        .build()
                )
            }

            EndPose.RIGHT -> {
                pose = when (StartPose.alliance) {
                    Alliance.RED -> endPoseRightRed
                    Alliance.BLUE -> endPoseRightBlue
                }
                drive.followTrajectorySequenceAsync(
                    drive.trajectorySequenceBuilder(drive.poseEstimate)
                        .lineToLinearHeading(pose)
                        .build()
                )
            }

            else -> {}
        }
        AutoHardware.updatePose(drive)
        //        encoderDrive(motorExtension, -autoExtension, 1);
    }
}
