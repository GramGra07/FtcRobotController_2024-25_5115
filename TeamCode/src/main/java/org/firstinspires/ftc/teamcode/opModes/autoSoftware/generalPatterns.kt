package org.firstinspires.ftc.teamcode.opModes.autoSoftware

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.teamcode.Enums.Alliance
import org.firstinspires.ftc.teamcode.Enums.AutoRandom
import org.firstinspires.ftc.teamcode.Enums.PathLong
import org.firstinspires.ftc.teamcode.Enums.Placement
import org.firstinspires.ftc.teamcode.Enums.StartDist
import org.firstinspires.ftc.teamcode.Enums.StartSide
import org.firstinspires.ftc.teamcode.Trajectories.backdrop.BackdropTrajectories
import org.firstinspires.ftc.teamcode.Trajectories.backdrop.BackdropTrajectories.backdropOffset
import org.firstinspires.ftc.teamcode.Trajectories.backdrop.BackdropTrajectories.blueMidOff
import org.firstinspires.ftc.teamcode.Trajectories.backdrop.BackdropTrajectories.endAngle
import org.firstinspires.ftc.teamcode.Trajectories.backdrop.BackdropTrajectories.forwardOffset
import org.firstinspires.ftc.teamcode.Trajectories.backdrop.BackdropTrajectories.startOffsetBlue
import org.firstinspires.ftc.teamcode.Trajectories.backdrop.BackdropTrajectories.startOffsetRed
import org.firstinspires.ftc.teamcode.Trajectories.spikeNavTraj.fwdLeft
import org.firstinspires.ftc.teamcode.Trajectories.spikeNavTraj.fwdRight
import org.firstinspires.ftc.teamcode.Trajectories.spikeNavTraj.midPiNav
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.StartPose
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.calcFlipPose
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.flipServo
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive

object generalPatterns {
    // method to go to the backdrop
    fun navToBackdrop_Place(drive: MecanumDrive, pathLong: PathLong, isCycling: Boolean) {
        if (!isCycling) {
            when (StartPose.alliance) {
                Alliance.RED -> when (StartPose.side) {
                    StartSide.LEFT -> drive.followTrajectorySequenceAsync(
                        BackdropTrajectories.redLong(
                            drive,
                            pathLong
                        )
                    )

                    StartSide.RIGHT -> drive.followTrajectorySequenceAsync(
                        BackdropTrajectories.redShort(
                            drive
                        )
                    )
                }

                Alliance.BLUE -> when (StartPose.side) {
                    StartSide.LEFT -> drive.followTrajectorySequenceAsync(
                        BackdropTrajectories.blueShort(
                            drive
                        )
                    )

                    StartSide.RIGHT -> drive.followTrajectorySequenceAsync(
                        BackdropTrajectories.blueLong(
                            drive,
                            pathLong
                        )
                    )
                }
            }
        } else {
            if (pathLong == PathLong.INSIDE) {
                when (StartPose.alliance) {
                    Alliance.RED -> drive.followTrajectorySequenceAsync(
                        BackdropTrajectories.redLong(
                            drive,
                            pathLong
                        )
                    )

                    Alliance.BLUE -> drive.followTrajectorySequenceAsync(
                        BackdropTrajectories.blueLong(
                            drive,
                            pathLong
                        )
                    )
                }
            } else {
                val baseX: Int
                val baseY: Int
                when (StartPose.alliance) {
                    Alliance.RED -> {
                        baseX = 58 + forwardOffset - backdropOffset
                        baseY = -32 - 2
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.poseEstimate)
                            .lineToLinearHeading(
                                Pose2d(
                                    -36.0,
                                    autoHardware.START_POSE.y + startOffsetRed,
                                    Math.toRadians(endAngle.toDouble())
                                )
                            )
                            .lineToLinearHeading(
                                Pose2d(
                                    autoHardware.START_POSE.x,
                                    autoHardware.START_POSE.y + startOffsetRed + 1,
                                    Math.toRadians(endAngle.toDouble())
                                )
                            )
                            .addSpatialMarker(Vector2d(-24.0, -36.0)) {
                                flipServo.calcFlipPose(30.0)
                            }
                            .splineToLinearHeading(
                                Pose2d(
                                    baseX.toDouble(),
                                    baseY.toDouble(),
                                    Math.toRadians(endAngle.toDouble())
                                ), Math.toRadians(endAngle.toDouble())
                            )
                            .build())
                    }

                    Alliance.BLUE -> {
                        baseX = 58 + forwardOffset - backdropOffset
                        baseY = 38 + blueMidOff
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.poseEstimate)
                            .lineToLinearHeading(
                                Pose2d(
                                    -36.0,
                                    autoHardware.START_POSE.y - startOffsetBlue,
                                    Math.toRadians(endAngle.toDouble())
                                )
                            )
                            .lineToLinearHeading(
                                Pose2d(
                                    autoHardware.START_POSE.x,
                                    autoHardware.START_POSE.y - startOffsetBlue - 1,
                                    Math.toRadians(endAngle.toDouble())
                                )
                            )
                            .addSpatialMarker(Vector2d(-24.0, -36.0)) {
                                flipServo.calcFlipPose(30.0)
                            }
                            .splineToLinearHeading(
                                Pose2d(
                                    baseX.toDouble(),
                                    baseY.toDouble(),
                                    Math.toRadians(endAngle)
                                ), Math.toRadians(endAngle)
                            )
                            .build())
                    }
                }
            }
        }
    }

    // drive and place first pixel
    fun SpikeNav(drive: MecanumDrive, pathLong: PathLong?) {
        val farSwingPoseRED = Pose2d(-54.0, -22.0, Math.toRadians(0.0))
        val farSwingPoseBLUE =
            Pose2d(farSwingPoseRED.x + 2, -farSwingPoseRED.y, farSwingPoseRED.heading)
        val placement: Placement
        val RedRight =
            HardwareConfig.Companion.startDist == StartDist.SHORT_SIDE && StartPose.alliance == Alliance.RED
        val BlueLeft =
            HardwareConfig.Companion.startDist == StartDist.SHORT_SIDE && StartPose.alliance == Alliance.BLUE
        val BlueRight =
            HardwareConfig.Companion.startDist == StartDist.LONG_SIDE && StartPose.alliance == Alliance.BLUE
        val RedLeft =
            HardwareConfig.Companion.startDist == StartDist.LONG_SIDE && StartPose.alliance == Alliance.RED
        placement =
            if (RedRight) Placement.RED_RIGHT else if (BlueLeft) Placement.BLUE_LEFT else if (BlueRight) Placement.BLUE_RIGHT else if (RedLeft) Placement.RED_LEFT else Placement.RED_RIGHT
        when (autoHardware.autonomousRandom) {
            AutoRandom.LEFT-> {
                when (placement) {
                    Placement.BLUE_RIGHT, Placement.RED_RIGHT, Placement.RED_LEFT -> drive.followTrajectorySequenceAsync(
                        fwdLeft(drive)
                    )

                    Placement.BLUE_LEFT -> drive.followTrajectorySequenceAsync(
                        drive.trajectorySequenceBuilder(drive.poseEstimate)
                            .splineToLinearHeading(
                                Pose2d(
                                    autoHardware.START_POSE.x + 13,
                                    autoHardware.START_POSE.y - 20,
                                    autoHardware.START_POSE.heading
                                ), autoHardware.START_POSE.heading
                            )
                            .build()
                    )
                }
                autoHardware.updatePose(drive)
                autoHardware.autoRandomReliable = AutoRandom.LEFT
            }

            AutoRandom.MID -> {
                when (placement) {
                    Placement.RED_RIGHT, Placement.BLUE_LEFT -> drive.followTrajectorySequenceAsync(
                        midPiNav(drive)
                    )

                    Placement.BLUE_RIGHT -> when (pathLong) {
                        PathLong.NONE, PathLong.OUTSIDE -> drive.followTrajectorySequenceAsync(
                            midPiNav(drive)
                        )

                        PathLong.INSIDE -> drive.followTrajectorySequenceAsync(
                            drive.trajectorySequenceBuilder(drive.poseEstimate)
                                .splineToLinearHeading(farSwingPoseBLUE, farSwingPoseBLUE.heading)
                                .build()
                        )

                        else -> {}
                    }

                    Placement.RED_LEFT -> when (pathLong) {
                        PathLong.NONE, PathLong.OUTSIDE -> drive.followTrajectorySequenceAsync(
                            midPiNav(drive)
                        )

                        PathLong.INSIDE -> drive.followTrajectorySequenceAsync(
                            drive.trajectorySequenceBuilder(drive.poseEstimate)
                                .splineToLinearHeading(farSwingPoseRED, farSwingPoseRED.heading)
                                .build()
                        )

                        else -> {}
                    }
                }
                autoHardware.updatePose(drive)
                autoHardware.autoRandomReliable = AutoRandom.MID
            }

            AutoRandom.RIGHT -> {
                when (placement) {
                    Placement.BLUE_RIGHT, Placement.RED_LEFT, Placement.BLUE_LEFT -> drive.followTrajectorySequenceAsync(
                        fwdRight(drive)
                    )

                    Placement.RED_RIGHT -> drive.followTrajectorySequenceAsync(
                        drive.trajectorySequenceBuilder(drive.poseEstimate)
                            .splineToLinearHeading(
                                Pose2d(
                                    autoHardware.START_POSE.x + 16,
                                    autoHardware.START_POSE.y + 17,
                                    autoHardware.START_POSE.heading
                                ), autoHardware.START_POSE.heading
                            )
                            .build()
                    )
                }
                autoHardware.updatePose(drive)
                autoHardware.autoRandomReliable = AutoRandom.RIGHT
            }

            else -> {}
        }
    }
}
