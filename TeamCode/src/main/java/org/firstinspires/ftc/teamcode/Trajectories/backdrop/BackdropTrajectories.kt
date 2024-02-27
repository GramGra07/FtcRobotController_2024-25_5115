package org.firstinspires.ftc.teamcode.Trajectories.backdrop

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.util.Angle.normDelta
import org.firstinspires.ftc.teamcode.Enums.AutoRandom
import org.firstinspires.ftc.teamcode.Enums.AutoRandom.MID
import org.firstinspires.ftc.teamcode.Enums.PathLong
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.AutoServoPositions.flipUp
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.calcFlipPose
//import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.flipServo
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.AutoHardware
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive
import org.firstinspires.ftc.teamcode.opModes.rr.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem

//@Config
object BackdropTrajectories {
    var endAngle = 0.0
    var offset = 8
    var startOffsetRed = 3
    var startOffsetBlue = 2
    var xOffset = 4
    var backdropOffset = 6
    var blueMidOff = 0
    var redMidOff = 0
    var forwardOffset = 0

    //    public static Pose2d backRed = new Pose2d(58 + forwardOffset, -32 - redMidOff - shiftOffset, Math.toRadians(endAngle));
    //    public static Pose2d backBlue = new Pose2d(58 + forwardOffset, 38 + blueMidOff - shiftOffset, Math.toRadians(endAngle));
    fun redShort(drive: MecanumDrive): TrajectorySequence {
        val baseX = 54 + forwardOffset //!
        val baseY = -32 - redMidOff - 1
        when (AutoHardware.autoRandomReliable) {
            MID -> return drive.trajectorySequenceBuilder(drive.poseEstimate)
                .lineToLinearHeading(
                    Pose2d(
                        baseX.toDouble(),
                        baseY.toDouble(),
                        Math.toRadians(endAngle.toDouble())
                    )
                )
                .build()

            AutoRandom.RIGHT -> return drive.trajectorySequenceBuilder(drive.poseEstimate)
                .lineToLinearHeading(
                    Pose2d(
                        baseX.toDouble(),
                        (baseY + ShiftTrajectories.rightShift).toDouble(),
                        Math.toRadians(
                            endAngle.toDouble()
                        )
                    )
                )
                .build()

            AutoRandom.LEFT -> return drive.trajectorySequenceBuilder(drive.poseEstimate)
                .lineToLinearHeading(
                    Pose2d(
                        baseX.toDouble(),
                        (baseY + ShiftTrajectories.leftShift).toDouble(),
                        Math.toRadians(
                            endAngle.toDouble()
                        )
                    )
                )
                .build()

            else -> {}
        }
        return drive.trajectorySequenceBuilder(drive.poseEstimate)
            .lineToLinearHeading(drive.poseEstimate)
            .build()
    }

    fun redLong(drive: MecanumDrive, pathLong: PathLong?, clawSubsystem: ClawSubsystem): TrajectorySequence? {
        val baseX = 58 + forwardOffset - backdropOffset
        var baseY = -32 - 2
        return when (pathLong) {
            PathLong.INSIDE -> {
                when (AutoHardware.autoRandomReliable) {
                    MID -> return drive.trajectorySequenceBuilder(drive.poseEstimate)
                        .addDisplacementMarker {
                            clawSubsystem.flipHigh()
                            clawSubsystem.update()
//                            flipServo.calcFlipPose((flipUp).toDouble())
                        }
                        .lineToLinearHeading(
                            Pose2d(
                                -52.0,
                                -10.0,
                                Math.toRadians(endAngle.toDouble())
                            )
                        )
                        .lineTo(Vector2d(36.0, -12.0))
                        .lineTo(Vector2d(36.0, -30.0)) //!
                        .splineToLinearHeading(
                            Pose2d(
                                baseX.toDouble(),
                                baseY.toDouble(),
                                Math.toRadians(endAngle.toDouble())
                            ), Math.toRadians(endAngle.toDouble())
                        )
                        .build()

                    AutoRandom.RIGHT -> return drive.trajectorySequenceBuilder(drive.poseEstimate)
                        .addDisplacementMarker {
                            clawSubsystem.flipUp()
                            clawSubsystem.update()
//                            flipServo.calcFlipPose((flipUp).toDouble())
                        }
                        .lineToLinearHeading(
                            Pose2d(
                                -52.0,
                                -10.0,
                                Math.toRadians(endAngle.toDouble())
                            )
                        )
                        .lineTo(Vector2d(36.0, -12.0))
                        .lineTo(Vector2d(36.0, -30.0)) //!
                        .splineToLinearHeading(
                            Pose2d(
                                baseX.toDouble(),
                                (baseY + ShiftTrajectories.rightShift).toDouble(),
                                Math.toRadians(endAngle.toDouble())
                            ), Math.toRadians(endAngle.toDouble())
                        )
                        .build()

                    AutoRandom.LEFT -> return drive.trajectorySequenceBuilder(drive.poseEstimate)
                        .addDisplacementMarker {
                            clawSubsystem.flipUp()
                            clawSubsystem.update()
//                            flipServo.calcFlipPose((flipUp).toDouble())
                        }
                        .turn(normDelta(AutoHardware.START_POSE.heading - drive.poseEstimate.heading))
                        .lineTo(
                            Vector2d(
                                drive.poseEstimate.x + 4,
                                -10.0
                            )
                        )
                        .lineToLinearHeading(
                            Pose2d(
                                36.0,
                                -12.0,
                                Math.toRadians(endAngle.toDouble())
                            )
                        )
                        .lineTo(Vector2d(36.0, -30.0)) //!
                        .splineToLinearHeading(
                            Pose2d(
                                baseX.toDouble(),
                                (baseY + ShiftTrajectories.leftShift).toDouble(),
                                Math.toRadians(endAngle.toDouble())
                            ), Math.toRadians(endAngle.toDouble())
                        )
                        .build()

                    else -> {}
                }
                baseY += 5
                when (AutoHardware.autoRandomReliable) {
                    MID -> return drive.trajectorySequenceBuilder(drive.poseEstimate)
                        .addDisplacementMarker {
                            clawSubsystem.flipUp()
                            clawSubsystem.update()
//                            flipServo.calcFlipPose((flipUp).toDouble())
                        }
                        .lineToLinearHeading(
                            Pose2d(
                                AutoHardware.START_POSE.x - xOffset,
                                AutoHardware.START_POSE.y + startOffsetRed,
                                Math.toRadians(endAngle.toDouble())
                            )
                        )
                        .lineToLinearHeading(
                            Pose2d(
                                -AutoHardware.START_POSE.x - offset,
                                AutoHardware.START_POSE.y + startOffsetRed,
                                Math.toRadians(endAngle.toDouble())
                            )
                        )
                        .splineToLinearHeading(
                            Pose2d(
                                baseX.toDouble(),
                                baseY.toDouble(),
                                Math.toRadians(endAngle.toDouble())
                            ), Math.toRadians(endAngle.toDouble())
                        )
                        .build()

                    AutoRandom.LEFT -> return drive.trajectorySequenceBuilder(drive.poseEstimate)
                        .addDisplacementMarker {
                            clawSubsystem.flipUp()
                            clawSubsystem.update()
//                            flipServo.calcFlipPose((flipUp).toDouble())
                        }
                        .lineToLinearHeading(
                            Pose2d(
                                AutoHardware.START_POSE.x - xOffset,
                                AutoHardware.START_POSE.y + startOffsetRed,
                                Math.toRadians(endAngle.toDouble())
                            )
                        )
                        .lineToLinearHeading(
                            Pose2d(
                                -AutoHardware.START_POSE.x - offset,
                                AutoHardware.START_POSE.y + startOffsetRed,
                                Math.toRadians(endAngle.toDouble())
                            )
                        )
                        .splineToLinearHeading(
                            Pose2d(
                                baseX.toDouble(),
                                (baseY + ShiftTrajectories.leftShift).toDouble(),
                                Math.toRadians(endAngle.toDouble())
                            ), Math.toRadians(endAngle.toDouble())
                        )
                        .build()

                    AutoRandom.RIGHT -> return drive.trajectorySequenceBuilder(drive.poseEstimate)
                        .addDisplacementMarker {
                            clawSubsystem.flipUp()
                            clawSubsystem.update()
//                            flipServo.calcFlipPose((flipUp).toDouble())
                        }
                        .lineToLinearHeading(
                            Pose2d(
                                AutoHardware.START_POSE.x - xOffset,
                                AutoHardware.START_POSE.y + startOffsetRed,
                                Math.toRadians(endAngle.toDouble())
                            )
                        )
                        .lineToLinearHeading(
                            Pose2d(
                                -AutoHardware.START_POSE.x - offset,
                                AutoHardware.START_POSE.y + startOffsetRed,
                                Math.toRadians(endAngle.toDouble())
                            )
                        )
                        .splineToLinearHeading(
                            Pose2d(
                                baseX.toDouble(),
                                (baseY + ShiftTrajectories.rightShift).toDouble(),
                                Math.toRadians(endAngle.toDouble())
                            ), Math.toRadians(endAngle.toDouble())
                        )
                        .build()

                    else -> {}
                }
                null
            }

            PathLong.OUTSIDE -> {
                baseY += 5
                when (AutoHardware.autoRandomReliable) {
                    MID -> return drive.trajectorySequenceBuilder(drive.poseEstimate)
                        .addDisplacementMarker {
                            clawSubsystem.flipUp()
                            clawSubsystem.update()
//                            flipServo.calcFlipPose((flipUp).toDouble())
                        }
                        .lineToLinearHeading(
                            Pose2d(
                                AutoHardware.START_POSE.x - xOffset,
                                AutoHardware.START_POSE.y + startOffsetRed,
                                Math.toRadians(endAngle.toDouble())
                            )
                        )
                        .lineToLinearHeading(
                            Pose2d(
                                -AutoHardware.START_POSE.x - offset,
                                AutoHardware.START_POSE.y + startOffsetRed,
                                Math.toRadians(endAngle.toDouble())
                            )
                        )
                        .splineToLinearHeading(
                            Pose2d(
                                baseX.toDouble(),
                                baseY.toDouble(),
                                Math.toRadians(endAngle.toDouble())
                            ), Math.toRadians(endAngle.toDouble())
                        )
                        .build()

                    AutoRandom.LEFT -> return drive.trajectorySequenceBuilder(drive.poseEstimate)
                        .addDisplacementMarker {
                            clawSubsystem.flipUp()
                            clawSubsystem.update()
//                            flipServo.calcFlipPose((flipUp).toDouble())
                        }
                        .lineToLinearHeading(
                            Pose2d(
                                AutoHardware.START_POSE.x - xOffset,
                                AutoHardware.START_POSE.y + startOffsetRed,
                                Math.toRadians(endAngle.toDouble())
                            )
                        )
                        .lineToLinearHeading(
                            Pose2d(
                                -AutoHardware.START_POSE.x - offset,
                                AutoHardware.START_POSE.y + startOffsetRed,
                                Math.toRadians(endAngle.toDouble())
                            )
                        )
                        .splineToLinearHeading(
                            Pose2d(
                                baseX.toDouble(),
                                (baseY + ShiftTrajectories.leftShift).toDouble(),
                                Math.toRadians(endAngle.toDouble())
                            ), Math.toRadians(endAngle.toDouble())
                        )
                        .build()

                    AutoRandom.RIGHT -> return drive.trajectorySequenceBuilder(drive.poseEstimate)
                        .addDisplacementMarker {
                            clawSubsystem.flipUp()
                            clawSubsystem.update()
//                            flipServo.calcFlipPose((flipUp).toDouble())
                        }
                        .lineToLinearHeading(
                            Pose2d(
                                AutoHardware.START_POSE.x - xOffset,
                                AutoHardware.START_POSE.y + startOffsetRed,
                                Math.toRadians(endAngle.toDouble())
                            )
                        )
                        .lineToLinearHeading(
                            Pose2d(
                                -AutoHardware.START_POSE.x - offset,
                                AutoHardware.START_POSE.y + startOffsetRed,
                                Math.toRadians(endAngle.toDouble())
                            )
                        )
                        .splineToLinearHeading(
                            Pose2d(
                                baseX.toDouble(),
                                (baseY + ShiftTrajectories.rightShift).toDouble(),
                                Math.toRadians(endAngle.toDouble())
                            ), Math.toRadians(endAngle.toDouble())
                        )
                        .build()

                    else -> {}
                }
                null
            }

            else -> null
        }
    }

    fun blueShort(drive: MecanumDrive): TrajectorySequence {
        val baseX = 58 + forwardOffset
        val baseY = 38 + blueMidOff
        when (AutoHardware.autoRandomReliable) {
            AutoRandom.MID -> return drive.trajectorySequenceBuilder(drive.poseEstimate)
                .lineToLinearHeading(
                    Pose2d(
                        baseX.toDouble(),
                        baseY.toDouble(),
                        Math.toRadians(endAngle.toDouble())
                    )
                )
                .build()

            AutoRandom.RIGHT -> return drive.trajectorySequenceBuilder(drive.poseEstimate)
                .lineToLinearHeading(
                    Pose2d(
                        baseX.toDouble(),
                        (baseY + ShiftTrajectories.rightShift).toDouble(),
                        Math.toRadians(
                            endAngle.toDouble()
                        )
                    )
                )
                .build()

            AutoRandom.LEFT -> return drive.trajectorySequenceBuilder(drive.poseEstimate)
                .lineToLinearHeading(
                    Pose2d(
                        baseX.toDouble(),
                        (baseY + ShiftTrajectories.leftShift).toDouble(),
                        Math.toRadians(
                            endAngle.toDouble()
                        )
                    )
                )
                .build()

            else -> {}
        }
        return drive.trajectorySequenceBuilder(drive.poseEstimate)
            .lineToLinearHeading(drive.poseEstimate)
            .build()
    }

    fun blueLong(drive: MecanumDrive, pathLong: PathLong?,clawSubsystem: ClawSubsystem): TrajectorySequence? {
        val baseX = 58 + forwardOffset - backdropOffset
        var baseY = 38 + blueMidOff
        return when (pathLong) {
            PathLong.INSIDE -> {
                when (AutoHardware.autoRandomReliable) {
                    AutoRandom.LEFT -> return drive.trajectorySequenceBuilder(drive.poseEstimate)
                        .addDisplacementMarker {
                            clawSubsystem.flipUp()
                            clawSubsystem.update()
//                            flipServo.calcFlipPose((flipUp).toDouble())
                        }
                        .lineToLinearHeading(
                            Pose2d(
                                drive.poseEstimate.x,
                                12.0,
                                Math.toRadians(endAngle.toDouble())
                            )
                        )
                        .lineTo(Vector2d(36.0, 12.0))
                        .lineTo(
                            Vector2d(
                                36.0,
                                30.0
                            )
                        ) //!might be able to remove this
                        .splineToLinearHeading(
                            Pose2d(
                                baseX.toDouble(),
                                (baseY + ShiftTrajectories.leftShift / 2).toDouble(),
                                Math.toRadians(endAngle.toDouble())
                            ), Math.toRadians(endAngle.toDouble())
                        )
                        .build()

                    AutoRandom.MID -> return drive.trajectorySequenceBuilder(drive.poseEstimate)
                        .addDisplacementMarker {
                            clawSubsystem.flipUp()
                            clawSubsystem.update()
//                            flipServo.calcFlipPose((flipUp).toDouble())
                        }
                        .lineToLinearHeading(
                            Pose2d(
                                -50.0,
                                12.0,
                                Math.toRadians(endAngle.toDouble())
                            )
                        )
                        .lineTo(Vector2d(36.0, 10.0))
                        .lineTo(Vector2d(36.0, 30.0)) //!
                        .splineToLinearHeading(
                            Pose2d(
                                baseX.toDouble(),
                                baseY.toDouble(),
                                Math.toRadians(endAngle.toDouble())
                            ), Math.toRadians(endAngle.toDouble())
                        )
                        .build()

                    AutoRandom.RIGHT -> return drive.trajectorySequenceBuilder(drive.poseEstimate)
                        .addDisplacementMarker {
                            clawSubsystem.flipUp()
                            clawSubsystem.update()
//                            flipServo.calcFlipPose((flipUp).toDouble())
                        }
                        .lineToLinearHeading(
                            Pose2d(
                                drive.poseEstimate.x,
                                12.0,
                                Math.toRadians(endAngle.toDouble())
                            )
                        )
                        .lineTo(Vector2d(36.0, 12.0))
                        .lineTo(Vector2d(36.0, 30.0)) //!
                        .splineToLinearHeading(
                            Pose2d(
                                baseX.toDouble(),
                                (baseY + ShiftTrajectories.rightShift - 2).toDouble(),
                                Math.toRadians(endAngle.toDouble())
                            ), Math.toRadians(endAngle.toDouble())
                        )
                        .build()

                    else -> {}
                }
                baseY -= 4
                when (AutoHardware.autoRandomReliable) {
                    AutoRandom.MID -> return drive.trajectorySequenceBuilder(drive.poseEstimate)
                        .addDisplacementMarker {
                            clawSubsystem.flipUp()
                            clawSubsystem.update()
//                            flipServo.calcFlipPose((flipUp).toDouble())
                        }
                        .lineToLinearHeading(
                            Pose2d(
                                AutoHardware.START_POSE.x,
                                AutoHardware.START_POSE.y - startOffsetBlue,
                                Math.toRadians(endAngle.toDouble())
                            )
                        )
                        .lineToLinearHeading(
                            Pose2d(
                                -AutoHardware.START_POSE.x - offset,
                                AutoHardware.START_POSE.y - startOffsetBlue,
                                Math.toRadians(endAngle.toDouble())
                            )
                        )
                        .lineTo(Vector2d(36.0, 30.0)) //!
                        .splineToLinearHeading(
                            Pose2d(
                                baseX.toDouble(),
                                baseY.toDouble(),
                                Math.toRadians(endAngle.toDouble())
                            ), Math.toRadians(endAngle.toDouble())
                        )
                        .build()

                    AutoRandom.LEFT -> return drive.trajectorySequenceBuilder(drive.poseEstimate)
                        .addDisplacementMarker {
                            clawSubsystem.flipUp()
                            clawSubsystem.update()
//                            flipServo.calcFlipPose((flipUp).toDouble())
                        }
                        .lineToLinearHeading(
                            Pose2d(
                                AutoHardware.START_POSE.x,
                                AutoHardware.START_POSE.y - startOffsetBlue,
                                Math.toRadians(endAngle.toDouble())
                            )
                        )
                        .lineToLinearHeading(
                            Pose2d(
                                -AutoHardware.START_POSE.x - offset,
                                AutoHardware.START_POSE.y - startOffsetBlue,
                                Math.toRadians(endAngle.toDouble())
                            )
                        )
                        .lineTo(Vector2d(36.0, 30.0)) //!
                        .splineToLinearHeading(
                            Pose2d(
                                baseX.toDouble(),
                                (baseY + ShiftTrajectories.leftShift).toDouble(),
                                Math.toRadians(endAngle.toDouble())
                            ), Math.toRadians(endAngle.toDouble())
                        )
                        .build()

                    AutoRandom.RIGHT -> return drive.trajectorySequenceBuilder(drive.poseEstimate)
                        .addDisplacementMarker {
                            clawSubsystem.flipUp()
                            clawSubsystem.update()
//                            flipServo.calcFlipPose((flipUp).toDouble())
                        }
                        .lineToLinearHeading(
                            Pose2d(
                                AutoHardware.START_POSE.x,
                                AutoHardware.START_POSE.y - startOffsetBlue,
                                Math.toRadians(endAngle.toDouble())
                            )
                        )
                        .lineToLinearHeading(
                            Pose2d(
                                -AutoHardware.START_POSE.x - offset,
                                AutoHardware.START_POSE.y - startOffsetBlue,
                                Math.toRadians(endAngle.toDouble())
                            )
                        )
                        .lineTo(Vector2d(36.0, 30.0)) //!
                        .splineToLinearHeading(
                            Pose2d(
                                baseX.toDouble(),
                                (baseY + ShiftTrajectories.rightShift).toDouble(),
                                Math.toRadians(endAngle.toDouble())
                            ), Math.toRadians(endAngle.toDouble())
                        )
                        .build()

                    else -> {}
                }
                null
            }

            PathLong.OUTSIDE -> {
                baseY -= 4
                when (AutoHardware.autoRandomReliable) {
                    AutoRandom.MID -> return drive.trajectorySequenceBuilder(drive.poseEstimate)
                        .addDisplacementMarker {
                            clawSubsystem.flipUp()
                            clawSubsystem.update()
//                            flipServo.calcFlipPose((flipUp).toDouble())
                        }
                        .lineToLinearHeading(
                            Pose2d(
                                AutoHardware.START_POSE.x,
                                AutoHardware.START_POSE.y - startOffsetBlue,
                                Math.toRadians(endAngle.toDouble())
                            )
                        )
                        .lineToLinearHeading(
                            Pose2d(
                                -AutoHardware.START_POSE.x - offset,
                                AutoHardware.START_POSE.y - startOffsetBlue,
                                Math.toRadians(endAngle.toDouble())
                            )
                        )
                        .lineTo(Vector2d(36.0, 30.0))
                        .splineToLinearHeading(
                            Pose2d(
                                baseX.toDouble(),
                                baseY.toDouble(),
                                Math.toRadians(endAngle.toDouble())
                            ), Math.toRadians(endAngle.toDouble())
                        )
                        .build()

                    AutoRandom.LEFT -> return drive.trajectorySequenceBuilder(drive.poseEstimate)
                        .addDisplacementMarker {
                            clawSubsystem.flipUp()
                            clawSubsystem.update()
//                            flipServo.calcFlipPose((flipUp).toDouble())
                        }
                        .lineToLinearHeading(
                            Pose2d(
                                AutoHardware.START_POSE.x,
                                AutoHardware.START_POSE.y - startOffsetBlue,
                                Math.toRadians(endAngle.toDouble())
                            )
                        )
                        .lineToLinearHeading(
                            Pose2d(
                                -AutoHardware.START_POSE.x - offset,
                                AutoHardware.START_POSE.y - startOffsetBlue,
                                Math.toRadians(endAngle.toDouble())
                            )
                        )
                        .lineTo(Vector2d(36.0, 30.0))
                        .splineToLinearHeading(
                            Pose2d(
                                baseX.toDouble(),
                                (baseY + ShiftTrajectories.leftShift).toDouble(),
                                Math.toRadians(endAngle.toDouble())
                            ), Math.toRadians(endAngle.toDouble())
                        )
                        .build()

                    AutoRandom.RIGHT -> return drive.trajectorySequenceBuilder(drive.poseEstimate)
                        .addDisplacementMarker {
                            clawSubsystem.flipUp()
                            clawSubsystem.update()
//                            flipServo.calcFlipPose((flipUp).toDouble())
                        }
                        .lineToLinearHeading(
                            Pose2d(
                                AutoHardware.START_POSE.x,
                                AutoHardware.START_POSE.y - startOffsetBlue,
                                Math.toRadians(endAngle.toDouble())
                            )
                        )
                        .lineToLinearHeading(
                            Pose2d(
                                -AutoHardware.START_POSE.x - offset,
                                AutoHardware.START_POSE.y - startOffsetBlue,
                                Math.toRadians(endAngle.toDouble())
                            )
                        )
                        .lineTo(Vector2d(36.0, 30.0))
                        .splineToLinearHeading(
                            Pose2d(
                                baseX.toDouble(),
                                (baseY + ShiftTrajectories.rightShift).toDouble(),
                                Math.toRadians(endAngle.toDouble())
                            ), Math.toRadians(endAngle.toDouble())
                        )
                        .build()

                    else -> {}
                }
                null
            }

            else -> null
        }
    }
}
