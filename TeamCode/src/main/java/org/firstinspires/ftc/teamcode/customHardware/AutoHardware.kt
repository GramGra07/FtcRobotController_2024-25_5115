package org.firstinspires.ftc.teamcode.customHardware

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
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
        val blueEndRight = Pose2d(-24.0, -10.0, Math.toRadians(180.0))
        val redEndLeft = Pose2d(24.0, 10.0, Math.toRadians(0.0))
        val redEndRight = Pose2d(24.0, -10.0, Math.toRadians(0.0))
        val blueHuman = Pose2d(-46.0, 54.0, Math.toRadians(90.0))
        val redHuman = Pose2d(46.0, -60.0, Math.toRadians(-90.0))
        val blueBasket = Pose2d(50.0, 50.0, Math.toRadians(135.0))
        val redBasket = Pose2d(-50.0, -50.0, Math.toRadians(45.0))
        val blueSpecimen = Pose2d(0.0, 36.0, blueStartRight.heading.toDouble())
        val redSpecimen = Pose2d(0.0, -30.0, redStartRight.heading.toDouble())
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
                        .splineToConstantHeading(
                            Vector2d(redSpecimen.position.x, redSpecimen.position.y),
                            redSpecimen.heading.toDouble()
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
                    drive.actionBuilder(redSpecimen)
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
                        .strafeTo(Vector2d(redSample.position.x - 5, redHuman.position.y))
                        .setTangent(redStartRight.heading.toDouble())
                        .splineToConstantHeading(
                            Vector2d(
                                redSample.position.x + 2,
                                redSample.position.y
                            ), redSample.heading.toDouble()
                        )
                        .strafeTo(Vector2d(redSample.position.x + 2, redHuman.position.y))

                        .build(),

                    InstantAction { runAction = false }
                ),
                uAction(driverAid, armSubsystem, scoringSubsystem),
                driverAid.daAction(listOf(Runnable { driverAid.collapse() }))
            )
        )

    }

    fun grabSpeci() {
        runAction = true
        runBlocking(
            ParallelAction(
                SequentialAction(
                    drive.actionBuilder(
                        Pose2d(
                            redSample.position.x + 1,
                            redHuman.position.y,
                            redSample.heading.toDouble()
                        )
                    )

                        .strafeToLinearHeading(
                            Vector2d(redHuman.position.x, redHuman.position.y),
                            redHuman.heading.toDouble()
                        )

                        .build(),
                    InstantAction { runAction = false }
                ),
                uAction(driverAid, armSubsystem, scoringSubsystem),
                driverAid.daAction(listOf(Runnable { driverAid.human() }))
            )
        )
    }

    fun placeSpeci() {
        runAction = true
        runBlocking(
            ParallelAction(
                SequentialAction(
                    drive.actionBuilder(
                        redHuman
                    )
                        .setTangent(Math.toRadians(180.0))
                        .splineToLinearHeading(
                            Pose2d(
                                redSpecimen.position.x,
                                redSpecimen.position.y,
                                redSpecimen.heading.toDouble()
                            ), redSpecimen.heading.toDouble()
                        )

                        .build(),
                    InstantAction { runAction = false }
                ),
                uAction(driverAid, armSubsystem, scoringSubsystem),
                driverAid.daAction(listOf(Runnable { driverAid.highSpecimen() }))
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
                    uAction(driverAid, armSubsystem, scoringSubsystem, 450.0),
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
                        .lineToY(blueEndRight.position.y)
                        .setTangent(blueEndRight.heading.toDouble())
                        .lineToXLinearHeading(
                            blueEndRight.position.x,
                            blueEndRight.heading.toDouble()
                        )

                        .build(),
                    InstantAction { runAction = false }
                ),
                uAction(driverAid, armSubsystem, scoringSubsystem),
                driverAid.daAction(listOf(Runnable { driverAid.collapse() }))
            )
        )
    }

    fun endHuman() {
        runAction = true
        runBlocking(
            ParallelAction(
                uAction(driverAid, armSubsystem, scoringSubsystem),
                driverAid.daAction(listOf(Runnable { driverAid.collapse() })),
                SequentialAction(
                    drive.actionBuilder(
                        redSpecimen
                    )
                        .strafeToLinearHeading(
                            Vector2d(redHuman.position.x, redHuman.position.y - 4),
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
    ) : Action {
        override fun run(packet: TelemetryPacket): Boolean {
            driverAid.update()
            armSubsystem.update()
            scoringSubsystem.update()
            packet.put(armSubsystem.pitchT.toString(), armSubsystem.pitchEncoder.currentPosition)
            packet.put(armSubsystem.extendTarget.toString(), armSubsystem.extendEncoder.getMost())
            return Companion.runAction || !armSubsystem.bothAtTarget(tolerance)
        }
    }

    fun uAction(
        driverAid: DriverAid,
        armSubsystem: ArmSubsystem,
        scoringSubsystem: ScoringSubsystem,
        tolerance: Double = 100.0
    ): Action {
        return updateAction(driverAid, armSubsystem, scoringSubsystem, tolerance)
    }
}