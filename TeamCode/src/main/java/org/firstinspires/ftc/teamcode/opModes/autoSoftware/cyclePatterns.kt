package org.firstinspires.ftc.teamcode.opModes.autoSoftware

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.teamcode.Enums.Alliance
import org.firstinspires.ftc.teamcode.Enums.PathLong
import org.firstinspires.ftc.teamcode.Trajectories.backdrop.BackdropTrajectories
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.StartPose
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.calcFlipPose
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.flipServo
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive

object cyclePatterns {
    // method to pick up from the stack of pixels
    fun pickFromSpot(drive: MecanumDrive, pathLong: PathLong?) {
        val clawAngle = 15
        when (pathLong) {
            PathLong.INSIDE -> when (StartPose.alliance) {
                Alliance.RED -> {
                    autoHardware.spot = Pose2d(-54.5, -10.0, Math.toRadians(180.0))
                    drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.poseEstimate)
                        .lineToSplineHeading(Pose2d(36.0, -12.0, autoHardware.spot!!.heading))
                        .lineToLinearHeading(autoHardware.spot)
                        .addSpatialMarker(Vector2d(-36.0, -10.0)) {
                            flipServo.calcFlipPose(clawAngle.toDouble())
                        }
                        .addDisplacementMarker {
                            ServoUtil.closeClaw(HardwareConfig.claw1)
                            ServoUtil.closeClaw(HardwareConfig.claw2)
                        }
                        .back(1.0)
                        .build()
                    )
                }

                Alliance.BLUE -> {
                    autoHardware.spot = Pose2d(-54.5, 10.1, Math.toRadians(180.0))
                    drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.poseEstimate)
                        .lineToLinearHeading(Pose2d(36.0, 12.0, autoHardware.spot!!.heading))
                        .lineToLinearHeading(autoHardware.spot)
                        .addSpatialMarker(Vector2d(-36.0, 10.0)) {
                            flipServo.calcFlipPose(clawAngle.toDouble())
                        }
                        .addDisplacementMarker {
                            ServoUtil.closeClaw(HardwareConfig.claw1)
                            ServoUtil.closeClaw(HardwareConfig.claw2)
                        }
                        .back(1.0)
                        .build())
                }
            }

            PathLong.OUTSIDE -> when (StartPose.alliance) {
                Alliance.RED -> {
                    autoHardware.spot = Pose2d(-57.0, -31.0, Math.toRadians(180.0))
                    drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.poseEstimate)
                        .lineToLinearHeading(
                            Pose2d(
                                autoHardware.START_POSE.x,
                                autoHardware.START_POSE.y + BackdropTrajectories.startOffsetRed,
                                autoHardware.spot!!.heading
                            )
                        )
                        .splineToLinearHeading(
                            Pose2d(
                                -36.0,
                                autoHardware.START_POSE.y + BackdropTrajectories.startOffsetRed,
                                autoHardware.spot!!.heading
                            ), autoHardware.spot!!.heading
                        )
                        .splineToLinearHeading(autoHardware.spot, autoHardware.spot!!.heading)
                        .addSpatialMarker(Vector2d(-36.0, -58.0)) {
                            flipServo.calcFlipPose(clawAngle.toDouble())
                        }
                        .addDisplacementMarker {
                            ServoUtil.closeClaw(HardwareConfig.claw1)
                            ServoUtil.closeClaw(HardwareConfig.claw2)
                        }
                        .back(1.0)
                        .build())
                }

                Alliance.BLUE -> {
                    autoHardware.spot = Pose2d(-55.0, 31.0, Math.toRadians(180.0))
                    drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.poseEstimate)
                        .lineToLinearHeading(
                            Pose2d(
                                autoHardware.START_POSE.x,
                                autoHardware.START_POSE.y - BackdropTrajectories.startOffsetBlue,
                                autoHardware.spot!!.heading
                            )
                        )
                        .splineToLinearHeading(
                            Pose2d(
                                -36.0,
                                autoHardware.START_POSE.y - BackdropTrajectories.startOffsetBlue - 1,
                                autoHardware.spot!!.heading
                            ), autoHardware.spot!!.heading
                        )
                        .splineToLinearHeading(autoHardware.spot, autoHardware.spot!!.heading)
                        .addSpatialMarker(Vector2d(-36.0, 58.0)) {
                            flipServo.calcFlipPose(clawAngle.toDouble())
                        }
                        .addDisplacementMarker {
                            ServoUtil.closeClaw(HardwareConfig.claw1)
                            ServoUtil.closeClaw(HardwareConfig.claw2)
                        }
                        .back(1.0)
                        .build())
                }
            }

            else -> {}
        }
    }
}
