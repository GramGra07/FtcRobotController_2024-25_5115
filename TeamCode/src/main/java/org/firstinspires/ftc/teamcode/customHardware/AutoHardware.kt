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
import org.firstinspires.ftc.teamcode.utilClass.storage.PoseStorage
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.DAVars
import kotlin.math.sqrt


class AutoHardware(
    opmode: LinearOpMode,
    startLocation: StartLocation,
    ahwMap: HardwareMap = opmode.hardwareMap,
) : HardwareConfig(opmode, true, startLocation, ahwMap) {
    var drive: MecanumDrive = MecanumDrive(ahwMap, startLocation.startPose)

    init {
//        super.initRobot(ahwMap, true, startLocation)
        telemetry.addData("Status", "Initialized")
//        telemetry.addData("Alliance", startLocation.alliance)
        telemetry.update()
        lastPose = startLocation.startPose
        autoSetup()
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
        val redStartLeft = Pose2d(-34.0, -63.0, Math.toRadians(90.0))
        val redStartRight = Pose2d(8.0, -63.0, Math.toRadians(90.0))
        val redEndLeft = Pose2d(24.0, 10.0, Math.toRadians(0.0))
        val redEndRight = Pose2d(-10.0, -10.0, Math.toRadians(0.0))
        val redHuman = Pose2d(46.0, -60.0, Math.toRadians(-90.0))
        val redBasket = Pose2d(-43.0, -43.0, Math.toRadians(45.0))
        val redSpecimen = Pose2d(0.0, -34.0, redStartRight.heading.toDouble())
        val redSample = Pose2d(60.0, -12.0, Math.toRadians(0.0))
        val redNeutralSample = Pose2d(-58.0, -40.0, redStartRight.heading.toDouble())


        var runAction = true
        var lastPose = PoseStorage.currentPose
        fun updateLastPose(drive: MecanumDrive) {
            PoseStorage.currentPose = drive.pose
            lastPose = drive.pose
        }
    }

    fun scorePreloadSpeci() {
        runAction = true
        DAVars.hSpecimenP = 950.0
        runBlocking(
            SequentialAction(
                ParallelAction(
                    SequentialAction(
                        drive.actionBuilder(lastPose)
                            .strafeToConstantHeading(
                                Vector2d(redSpecimen.position.x, redSpecimen.position.y),
                            )
                            .build(),
                        endAction()
                    ),
                    driverAid.daAction(listOf(Runnable { driverAid.highSpecimen() })),
                    uAction(driverAid, armSubsystem, scoringSubsystem)
                ),
                scoringSubsystem.servoAction(
                    listOf(
                        Runnable { scoringSubsystem.openClaw() },
                    )
                ),
                InstantAction{DAVars.hSpecimenP = 850.0}
            )
        )
    }

    fun moveOneSpeci() {
        runAction = true
        runBlocking(
            SequentialAction(
            ParallelAction(
                uAction(driverAid, armSubsystem, scoringSubsystem),
                SequentialAction(
                    SleepAction(0.5),
                    ParallelAction(
                        driverAid.daAction(listOf(Runnable { driverAid.collapse() })),
                        scoringSubsystem.servoAction(listOf(Runnable { scoringSubsystem.setPitchHigh() }))
                    )
                ),
                SequentialAction(
                    drive.actionBuilder(lastPose)
                        .lineToY(redHuman.position.y + 5)
                        .build(),
                    endAction()
                )
            ),
                InstantAction { runAction = true },
                ParallelAction(
                    SequentialAction(
                        drive.actionBuilder(lastPose)
                            .strafeToLinearHeading(Vector2d(30.0,-36.0), Math.toRadians(25.0))
                        .build(),
                        endAction()
                    ),
                    ParallelAction(
                        driverAid.daAction(listOf(Runnable { driverAid.pickup() })),
                        uAction(driverAid, armSubsystem, scoringSubsystem),
                    )
                ),
                InstantAction { runAction = true },
                ParallelAction(
                    SequentialAction(
                        drive.actionBuilder(lastPose)
                            .turnTo(Math.toRadians(335.0))
                            .build(),
                        endAction()
                    ),
                        uAction(driverAid, armSubsystem, scoringSubsystem),
                ),
                ParallelAction(
                    driverAid.daAction(listOf(Runnable { driverAid.collapse() })),
                    uAction(driverAid, armSubsystem, scoringSubsystem)
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
                    drive.actionBuilder(lastPose)
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

                    endAction()
                ),
                uAction(driverAid, armSubsystem, scoringSubsystem),
            )
        )

    }

    fun grabSpeci(wait: Boolean? = null) {
        runAction = true
        runBlocking(
            SequentialAction(
                ParallelAction(
                    SequentialAction(
                        SleepAction(0.5),
                        driverAid.daAction(listOf(Runnable { driverAid.human() }))
                    ),
                    SequentialAction(
                        drive.actionBuilder(lastPose)
                            .lineToY(redHuman.position.y + 15)
                            .strafeToLinearHeading(
                                Vector2d(redHuman.position.x, redHuman.position.y + 8),
                                redHuman.heading.toDouble()
                            ).build(),
                        endAction()
                    ),
                    uAction(driverAid, armSubsystem, scoringSubsystem),
                ),
            )
        )
        if (wait != null && wait) {
            runBlocking(
                SleepAction(2.0)
            )
        }
        runAction = true
        runBlocking(
            SequentialAction(
                ParallelAction(
                    SequentialAction(
                        drive.actionBuilder(
                            lastPose
                        )
                            .lineToY(redHuman.position.y)
                            .stopAndAdd(scoringSubsystem.servoAction(listOf(Runnable { scoringSubsystem.closeClaw() })))
                            .build(),
                        endAction()
                    ),
                ),
                ParallelAction(
                    scoringSubsystem.servoAction(listOf(Runnable { scoringSubsystem.closeClaw() })),
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
                        lastPose
                    )
                        .setTangent(Math.toRadians(180.0))
                        .splineToLinearHeading(
                            Pose2d(
                                redSpecimen.position.x + offset,
                                redSpecimen.position.y,
                                redSpecimen.heading.toDouble()
                            ), redSpecimen.heading.toDouble()
                        )
                        .build(),
                    endAction()
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

    fun collapse() {
        runAction = false
        runBlocking(
            ParallelAction(
                driverAid.daAction(listOf(Runnable { driverAid.collapse() })),
                uAction(driverAid, armSubsystem, scoringSubsystem, 400.0)
            )
        )
    }

    fun hiBask() {
        runAction = false
        runBlocking(
            ParallelAction(
                driverAid.daAction(listOf(Runnable { driverAid.highBasket() })),
                uAction(driverAid, armSubsystem, scoringSubsystem)
            )
        )
    }


    fun scorePreloadSample() {
        runAction = true
        DAVars.hBasketP = 1850.0
        runBlocking(
                SequentialAction(
                    ParallelAction(
                        SequentialAction(
                            drive.actionBuilder(
                                lastPose
                            )
                                .setTangent(Math.toRadians(180.0))
                                .strafeToLinearHeading(
                                    Vector2d(redBasket.position.x, redBasket.position.y),
                                    Math.toRadians(45.0)
                                )
                                .build(),
                                    endAction()
                        ),
                        driverAid.daAction(listOf(Runnable { driverAid.highBasket() })),
                        uAction(driverAid, armSubsystem, scoringSubsystem, 450.0),
                    ),
                    InstantAction { runAction = true },
                    ParallelAction(
                        uAction(driverAid, armSubsystem, scoringSubsystem, 450.0),
                        SequentialAction(
                            scoringSubsystem.servoAction(
                                listOf(
                                    Runnable { scoringSubsystem.setPitchHigh() },
                                )
                            ),

                            SleepAction(0.2),

                            scoringSubsystem.servoAction(
                                listOf(
                                    Runnable { scoringSubsystem.openClaw() },
                                )
                            ),
                            SleepAction(0.2),

                            scoringSubsystem.servoAction(
                                listOf(
                                    Runnable { scoringSubsystem.setPitchMed() },
                                )
                            ),
                            InstantAction { runAction = false },
                        )
                    ),
                    InstantAction{DAVars.hBasketP = 1725.0},
                )
            )
    }
    fun Pose2d.distanceTo(pose2d: Pose2d): Double {
        val deltaX = pose2d.position.x - this.position.x
        val deltaY = pose2d.position.y - this.position.y
        return sqrt(deltaX * deltaX + deltaY * deltaY)
    }
    fun calculateExtend(pose2d: Pose2d,newPose: Pose2d): Double {
        val distance = pose2d.distanceTo(newPose)
        return distance
    }
    enum class SampleLocation {
        LEFT, CENTER, RIGHT
    }

    fun getSample(location: SampleLocation) {
        runAction = true
        val turnAngle = when (location) {
            SampleLocation.LEFT -> 152.0
            SampleLocation.CENTER -> 135.0
            SampleLocation.RIGHT -> 105.0
        }
        val offset = when (location) {
            SampleLocation.LEFT -> 0
            SampleLocation.CENTER -> -14
            SampleLocation.RIGHT -> -10
        }
        val pose = when (location) {
            SampleLocation.LEFT -> Pose2d(-68.0, -25.0, Math.toRadians(90.0))
            SampleLocation.CENTER -> Pose2d(-58.0, -25.0, Math.toRadians(90.0))
            SampleLocation.RIGHT -> Pose2d(-48.0, -25.0, Math.toRadians(90.0))
        }
        DAVars.pickUpE = (calculateExtend(lastPose, pose)+offset)*armSubsystem.ticksPerInchExtend
        runBlocking(
            SequentialAction(
                driverAid.daAction(listOf(Runnable { driverAid.pickup() })),
                ParallelAction(
                    SequentialAction(
                        drive.actionBuilder(
                            lastPose
                        )
                            .turnTo(Math.toRadians(turnAngle))
                            .build(),
                        endAction()
                    ),
                    uAction(driverAid, armSubsystem, scoringSubsystem),
                ),
                scoringSubsystem.servoAction(
                    listOf(
                        Runnable { scoringSubsystem.setRotateCenter() },
                    )
                ),
                scoringSubsystem.servoAction(
                    listOf(
                        Runnable { scoringSubsystem.setPitchLow() },
                    )
                ),
                SleepAction(0.2),

                scoringSubsystem.servoAction(
                    listOf(
                        Runnable { scoringSubsystem.closeClaw() },
                    )
                ),
                SleepAction(0.2),
                driverAid.daAction(listOf(Runnable { driverAid.collapse() })),
                ParallelAction(
                    scoringSubsystem.servoAction(
                        listOf(
                            Runnable { scoringSubsystem.setPitchMed() },
                        )
                    ),
                    uAction(driverAid, armSubsystem, scoringSubsystem,250.0),
                ),
                InstantAction{ DAVars.pickUpE = 2000.0},
            )
        )
    }

    fun scoreSample() {
        runAction = true
        DAVars.hBasketP = 1850.0
        runBlocking(
            SequentialAction(
                ParallelAction(
                    SequentialAction(
                        drive.actionBuilder(
                            lastPose
                        )
                            .turnTo(Math.toRadians(45.0))
                            .build(),
                        endAction()
                    ),
                    driverAid.daAction(listOf(Runnable { driverAid.highBasket() })),
                    uAction(driverAid, armSubsystem, scoringSubsystem, 450.0),
                ),
                InstantAction { runAction = true },
                ParallelAction(
                    uAction(driverAid, armSubsystem, scoringSubsystem, 450.0),
                    SequentialAction(
                        scoringSubsystem.servoAction(
                            listOf(
                                Runnable { scoringSubsystem.setPitchHigh() },
                            )
                        ),

                        SleepAction(0.2),

                        scoringSubsystem.servoAction(
                            listOf(
                                Runnable { scoringSubsystem.openClaw() },
                            )
                        ),
                        SleepAction(0.2),

                        scoringSubsystem.servoAction(
                            listOf(
                                Runnable { scoringSubsystem.setPitchMed() },
                            )
                        ),
                        InstantAction { runAction = false },
                    )
                ),
                InstantAction{DAVars.hBasketP = 1725.0},
            )
        )
    }

    fun endHuman() {
        runAction = true
        runBlocking(
            ParallelAction(
                uAction(driverAid, armSubsystem, scoringSubsystem),
                SequentialAction(
                    SleepAction(0.5), driverAid.daAction(listOf(Runnable { driverAid.collapse() }))
                ),
                SequentialAction(
                    drive.actionBuilder(
                        lastPose
                    )
                        .lineToY(redHuman.position.y + 10)
                        .strafeToLinearHeading(
                            Vector2d(redHuman.position.x, redHuman.position.y),
                            redHuman.heading.toDouble()
                        )
                        .build(),

                    endAction()
                )
            )
        )
    }

    fun endAction(): Action {
        return endAct(drive)
    }

    class endAct(val drive: MecanumDrive) : Action {
        override fun run(packet: TelemetryPacket): Boolean {
            runAction = false
            updateLastPose(this.drive)
            return false
        }
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
            return Companion.runAction || (!driverAid.isDone(tolerance))
        }
    }

    fun uAction(
        driverAid: DriverAid,
        armSubsystem: ArmSubsystem,
        scoringSubsystem: ScoringSubsystem,
        tolerance: Double = 100.0,
    ): Action {
        return updateAction(driverAid, armSubsystem, scoringSubsystem, tolerance)
    }


    fun testArm() {
        runAction = true
        runBlocking(
            ParallelAction(
                driverAid.daAction(listOf(Runnable { driverAid.highSpecimen() })),
                uAction(driverAid, armSubsystem, scoringSubsystem)
            )
        )
    }
}