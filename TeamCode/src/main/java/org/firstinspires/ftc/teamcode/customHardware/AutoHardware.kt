package org.firstinspires.ftc.teamcode.customHardware

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.followers.roadRunner.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.DriverAid
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.ArmSubsystem
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.ScoringSubsystem


class AutoHardware(
    opmode: LinearOpMode,
    startLocation: StartLocation,
    ahwMap: HardwareMap = opmode.hardwareMap,
) : HardwareConfig(opmode, true, startLocation, ahwMap) {
    var drive: MecanumDrive = MecanumDrive(ahwMap, startLocation.startPose)

    init {
//        super.initRobot(ahwMap, true, startLocation)
        telemetry.addData("Status", "Initialized")
        telemetry.addData("Alliance", startLocation.alliance)
        telemetry.update()
        opmode.waitForStart()
        timer.reset()
    }

    fun loop() {
        updateAll()
    }

    fun updateAll() {
        driverAid.update()
        armSubsystem.update()
        scoringSubsystem.update()
    }

    fun autoSetup() {
        this.scoringSubsystem.setup()
    }

    companion object {
        val blueStartLeft = Pose2d(-8.0, 63.0, Math.toRadians(-90.0))
        val blueStartRight = Pose2d(8.0, 63.0, Math.toRadians(-90.0))
        val redStartLeft = Pose2d(-8.0, -63.0, Math.toRadians(90.0))
        val redStartRight = Pose2d(8.0, -63.0, Math.toRadians(90.0))// used to be 135-72
        val blueEndLeft = Pose2d(-24.0, 10.0, Math.toRadians(180.0))
        val blueEndRight = Pose2d(-10.0, -10.0, Math.toRadians(180.0))
        val redEndLeft = Pose2d(24.0, 10.0, Math.toRadians(0.0))
        val redEndRight = Pose2d(24.0, -10.0, Math.toRadians(0.0))
        val blueHuman = Pose2d(-46.0, 54.0, Math.toRadians(90.0))
        val redHuman = Pose2d(46.0, -60.0, Math.toRadians(-90.0))
        val blueBasket = Pose2d(50.0, 50.0, Math.toRadians(135.0))
        val redBasket = Pose2d(-38.0, -48.0, Math.toRadians(45.0))
        val blueSpecimen = Pose2d(0.0, 36.0, blueStartRight.heading.toDouble())
        val redSpecimen = Pose2d(0.0, -28.0, redStartRight.heading.toDouble())
        val blueSample = Pose2d(-60.0, 12.0, Math.toRadians(0.0))
        val redSample = Pose2d(60.0, -12.0, Math.toRadians(0.0))
        val blueNeutralSample = Pose2d(56.0, 12.0, blueStartRight.heading.toDouble())
        val redNeutralSample = Pose2d(-58.0, -40.0, redStartRight.heading.toDouble())


        var runAction = true
    }

    fun scorePreloadSpeci() {
        runAction = true
        runBlocking(
            ParallelAction(
                SequentialAction(
                    drive.actionBuilder(redStartRight)
                        .strafeToConstantHeading(
                            Vector2d(redSpecimen.position.x - 2, redSpecimen.position.y),
                        )
                        .build(),
                    InstantAction { runAction = false }
                ),
                driverAid.daAction(listOf(Runnable { driverAid.highSpecimen() })),
                uAction(driverAid, armSubsystem, scoringSubsystem)
            ),
        )
        runBlocking(
            scoringSubsystem.servoAction(
                listOf(
                    Runnable { scoringSubsystem.openClaw() },
                )
            )
        )
    }

    fun moveAllSpeci() {
        runAction = true
        runBlocking(
            ParallelAction(
                SequentialAction(
                    SleepAction(0.5),
                    ParallelAction(
                        driverAid.daAction(listOf(Runnable { driverAid.collapse() })),
                        scoringSubsystem.servoAction(listOf(Runnable { scoringSubsystem.setPitchHigh() }))
                    )
                ),
                SequentialAction(
                    drive.actionBuilder(redSpecimen)
                        .lineToY(redHuman.position.y + 10)
                        .setTangent(Math.toRadians(-90.0))
                        .splineToLinearHeading(
                            Pose2d(36.0, -24.0, redSample.heading.toDouble()),
                            redSpecimen.heading.toDouble()
                        )
                        .splineToConstantHeading(
                            Vector2d(redSample.position.x - 16.0, redSample.position.y),
                            redSample.heading.toDouble()
                        )
                        .setTangent(redSpecimen.heading.toDouble())
                        .lineToY(redHuman.position.y)
                        .setTangent(redStartLeft.heading.toDouble())
                        .splineToLinearHeading(
                            Pose2d(
                                redSample.position.x - 5,
                                redSample.position.y,
                                redSample.heading.toDouble()
                            ), redSample.heading.toDouble()
                        )
                        .lineToY(redHuman.position.y)
                        .setTangent(redStartRight.heading.toDouble())
                        .splineToConstantHeading(
                            Vector2d(
                                redSample.position.x + 4,
                                redSample.position.y
                            ), redSample.heading.toDouble()
                        )
                        .lineToY(redHuman.position.y)

                        .build(),

                    InstantAction { runAction = false }
                ),
                uAction(driverAid, armSubsystem, scoringSubsystem),
            )
        )

    }

    fun grabSpeci(from: Pose2d) {
        runAction = true
        runBlocking(
            SequentialAction(
                ParallelAction(
                    SequentialAction(
                        SleepAction(0.5),
                        driverAid.daAction(listOf(Runnable { driverAid.human() }))
                    ),
                    SequentialAction(
                        drive.actionBuilder(from).lineToY(redHuman.position.y + 10)
                            .strafeToLinearHeading(
                                Vector2d(redHuman.position.x, redHuman.position.y + 8),
                                redHuman.heading.toDouble()
                            ).build(),
                        InstantAction { runAction = false }
                    ),
                    uAction(driverAid, armSubsystem, scoringSubsystem),
                ),
            )
        )
        runAction = true
        runBlocking(
            SequentialAction(
                ParallelAction(
                    SequentialAction(
                        drive.actionBuilder(
                            Pose2d(
                                Vector2d(redHuman.position.x, redHuman.position.y + 8),
                                redHuman.heading.toDouble()
                            )
                        )
                            .lineToY(redHuman.position.y)
                            .stopAndAdd(scoringSubsystem.servoAction(listOf(Runnable { scoringSubsystem.closeClaw() })))
                            .build(),
                        InstantAction { runAction = false },
                    ),
//                    uAction(driverAid, armSubsystem, scoringSubsystem),
                ),
                ParallelAction(
                    scoringSubsystem.servoAction(listOf(Runnable { scoringSubsystem.closeClaw() })),
//                    uAction(driverAid, armSubsystem, scoringSubsystem)
                )
            )
        )
    }

    fun placeSpeci(offset: Int) {
        runAction = true
        runBlocking(
            ParallelAction(
                SequentialAction(
                    SleepAction(1.0),
                    driverAid.daAction(listOf(Runnable { driverAid.highSpecimen() }))
                ),
                SequentialAction(
                    drive.actionBuilder(
                        redHuman
                    )
                        .setTangent(Math.toRadians(180.0))
                        .splineToLinearHeading(
                            Pose2d(
                                redSpecimen.position.x + offset,
                                redHuman.position.y + 20,
                                redSpecimen.heading.toDouble()
                            ), redSpecimen.heading.toDouble()
                        )
                        .lineToY(redHuman.position.y + 25)

                        .build(),
                    InstantAction { runAction = false }
                ),
                uAction(driverAid, armSubsystem, scoringSubsystem),
            )
        )
        runBlocking(
            scoringSubsystem.servoAction(
                listOf(
                    Runnable { scoringSubsystem.openClaw() },
                )
            )
        )
    }

    fun scorePreloadSample() {
        runAction = true
        runBlocking(
            SequentialAction(
                ParallelAction(
                    SequentialAction(
                        drive.actionBuilder(
                            redStartLeft
                        )
                            .setTangent(Math.toRadians(180.0))
                            .strafeToLinearHeading(
                                Vector2d(redBasket.position.x, redBasket.position.y),
                                Math.toRadians(45.0)
                            )
                            .build(),
                        InstantAction { runAction = false }
                    ),
                    uAction(driverAid, armSubsystem, scoringSubsystem, 450.0),
                    driverAid.daAction(listOf(Runnable { driverAid.highBasket() }))
                ),
            )
        )
        runAction = true
//        runBlocking(
//            ParallelAction(
//                SequentialAction(
//                    SleepAction(1.0), InstantAction { runAction = false }),
//                uAction(driverAid, armSubsystem, scoringSubsystem, 100.0, true)
//            )
//        )
        runBlocking(
            scoringSubsystem.servoAction(
                listOf(
                    Runnable { scoringSubsystem.setPitchHigh() },
                )
            ),
        )
        runBlocking(
            SleepAction(1.0)
        )
        runBlocking(
            scoringSubsystem.servoAction(
                listOf(
                    Runnable { scoringSubsystem.openClaw() },
                )
            )
        )
//        runBlocking(
//            ParallelAction(
//                SequentialAction(
//                    SleepAction(1.0), InstantAction { runAction = false }),
//                uAction(driverAid, armSubsystem, scoringSubsystem, 100.0, true)
//            )
//        )
    }

    fun getSampleR() {

        runAction = true
        runBlocking(
            SequentialAction(
                ParallelAction(
                    SequentialAction(
                        drive.actionBuilder(
                            redBasket
                        ).setTangent(Math.toRadians(180.0))
                            .strafeToLinearHeading(
                                Vector2d(
                                    redNeutralSample.position.x,
                                    redNeutralSample.position.y
                                ), Math.toRadians(50.0)
                            )

                            .build(),
                        InstantAction { runAction = false },
                    ),
                    uAction(driverAid, armSubsystem, scoringSubsystem),
                    driverAid.daAction(listOf(Runnable { driverAid.pickup() })),
                    scoringSubsystem.servoAction(
                        listOf(
                            Runnable { scoringSubsystem.setPitchLow() },
                        )
                    )
                ),
            )
        )
        runBlocking(
            scoringSubsystem.servoAction(
                listOf(
                    Runnable { scoringSubsystem.closeClaw() },
                )
            )
        )
    }

    fun getSampleM() {

        runAction = true
        runBlocking(
            SequentialAction(
                ParallelAction(
                    SequentialAction(
                        drive.actionBuilder(
                            redBasket
                        ).setTangent(Math.toRadians(180.0))
                            .splineToLinearHeading(redNeutralSample, Math.toRadians(0.0))
                            .build(),
                        InstantAction { runAction = false },
                    ),
                    uAction(driverAid, armSubsystem, scoringSubsystem),
                    driverAid.daAction(listOf(Runnable { driverAid.pickup() })),
                    scoringSubsystem.servoAction(
                        listOf(
                            Runnable { scoringSubsystem.setPitchLow() },
                        )
                    )
                ),
            )
        )
        runBlocking(
            scoringSubsystem.servoAction(
                listOf(
                    Runnable { scoringSubsystem.closeClaw() },
                )
            )
        )
    }

    fun getSampleL() {

        runAction = true
        runBlocking(
            SequentialAction(
                ParallelAction(
                    SequentialAction(
                        drive.actionBuilder(
                            redBasket
                        ).setTangent(Math.toRadians(180.0))
                            .strafeToLinearHeading(
                                Vector2d(
                                    redNeutralSample.position.x,
                                    redNeutralSample.position.y
                                ), Math.toRadians(130.0)
                            )
                            .build(),
                        InstantAction { runAction = false },
                    ),
                    uAction(driverAid, armSubsystem, scoringSubsystem),
                    driverAid.daAction(listOf(Runnable { driverAid.pickup() })),
                    scoringSubsystem.servoAction(
                        listOf(
                            Runnable { scoringSubsystem.setPitchLow() },
                        )
                    )
                ),
            )
        )
        runBlocking(
            scoringSubsystem.servoAction(
                listOf(
                    Runnable { scoringSubsystem.closeClaw() },
                )
            )
        )
    }

    fun scoreSample(from: Pose2d) {
        runAction = true
        runBlocking(
            SequentialAction(
                ParallelAction(
                    SequentialAction(
                        drive.actionBuilder(
                            from
                        )
                            .strafeToLinearHeading(
                                Vector2d(redBasket.position.x, redBasket.position.y),
                                Math.toRadians(45.0)
                            )
                            .build(),
                        InstantAction { runAction = false },
                    ),
                    uAction(driverAid, armSubsystem, scoringSubsystem, 250.0),
                    driverAid.daAction(listOf(Runnable { driverAid.highBasket() })),
                ),
            )

        )
        runBlocking(
            scoringSubsystem.servoAction(
                listOf(
                    Runnable { scoringSubsystem.setPitchHigh() },
                )
            ),
        )
        runBlocking(
            scoringSubsystem.servoAction(
                listOf(
                    Runnable { scoringSubsystem.openClaw() },
                )
            )
        )
    }

    fun end(from: Pose2d) {
        runAction = true
        runBlocking(
            ParallelAction(
                SequentialAction(
                    drive.actionBuilder(
                        from
                    )
                        .strafeTo(Vector2d(-36.0, -31.0))
                        .setTangent(Math.toRadians(90.0))
                        .lineToY(blueEndRight.position.y + 2)
                        .setTangent(blueEndRight.heading.toDouble())
                        .lineToXLinearHeading(
                            blueEndRight.position.x,
                            blueEndRight.heading.toDouble()
                        )

                        .build(),
                    InstantAction { runAction = false }
                ),
                uAction(driverAid, armSubsystem, scoringSubsystem),
                driverAid.daAction(listOf(Runnable { driverAid.auto() }))
            )
        )
    }

    fun endHuman(from: Pose2d = redSpecimen) {
        runAction = true
        runBlocking(
            ParallelAction(
                uAction(driverAid, armSubsystem, scoringSubsystem),
                SequentialAction(
                    SleepAction(0.5), driverAid.daAction(listOf(Runnable { driverAid.collapse() }))
                ),
                SequentialAction(
                    drive.actionBuilder(
                        from
                    )
                        .lineToY(redHuman.position.y + 10)
                        .strafeToLinearHeading(
                            Vector2d(redHuman.position.x, redHuman.position.y),
                            redHuman.heading.toDouble()
                        )
                        .build(),

                    InstantAction { runAction = false }
                )
            )
        )
    }

    class updateAction(
        val driverAid: DriverAid,
        val armSubsystem: ArmSubsystem,
        val scoringSubsystem: ScoringSubsystem,
        val tolerance: Double = 100.0,
        val load: Boolean = false
    ) : Action {
        override fun run(packet: TelemetryPacket): Boolean {
            driverAid.update()
            armSubsystem.update()
            scoringSubsystem.update()
            packet.put(armSubsystem.pitchT.toString(), armSubsystem.pitchEncoder.currentPosition)
            packet.put(armSubsystem.extendTarget.toString(), armSubsystem.extendEncoder.getMost())
            return if (!load) {
                Companion.runAction || !armSubsystem.bothAtTarget(tolerance)
            } else {
                Companion.runAction
            }
        }
    }

    fun uAction(
        driverAid: DriverAid,
        armSubsystem: ArmSubsystem,
        scoringSubsystem: ScoringSubsystem,
        tolerance: Double = 100.0,
        load: Boolean = false
    ): Action {
        return updateAction(driverAid, armSubsystem, scoringSubsystem, tolerance)
    }
}