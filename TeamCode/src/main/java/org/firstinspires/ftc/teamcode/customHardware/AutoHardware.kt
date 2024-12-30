package org.firstinspires.ftc.teamcode.customHardware

import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.followers.roadRunner.MecanumDrive
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.DAVars.hSpecimenP


class AutoHardware(
    opmode: LinearOpMode,
    startLocation: StartLocation,
    ahwMap: HardwareMap = opmode.hardwareMap,
) : HardwareConfig(opmode, true, startLocation, ahwMap) {
    lateinit var drive: MecanumDrive

    init {
        super.initRobot(ahwMap, true, startLocation)
        drive = MecanumDrive(ahwMap, startLocation.startPose)
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
        val redHuman = Pose2d(46.0, -54.0, Math.toRadians(-90.0))
        val blueBasket = Pose2d(50.0, 50.0, Math.toRadians(135.0))
        val redBasket = Pose2d(-50.0, -50.0, Math.toRadians(45.0))
        val blueSpecimen = Pose2d(0.0, 36.0, blueStartRight.heading.toDouble())
        val redSpecimen = Pose2d(0.0, -36.0, redStartRight.heading.toDouble())
        val blueSample = Pose2d(-60.0, 12.0, Math.toRadians(0.0))
        val redSample = Pose2d(60.0, -12.0, Math.toRadians(0.0))
        val blueNeutralSample = Pose2d(56.0, 12.0, blueStartRight.heading.toDouble())
        val redNeutralSample = Pose2d(-56.0, -12.0, redStartRight.heading.toDouble())
    }

//    fun goToSpecimenBCurve(): BezierCurve {
//        return BezierCurve(
//            PoseStorage.currentPoint,
//            Point(24.0, 72.0),
//            blueSpecimen
//        )
//    }
//
//    fun goToBasketBCurve(): BezierCurve {
//        return BezierCurve(
//            PoseStorage.currentPoint,
//            Point(36.0, 120.0),
//            blueBasket
//        )
//    }
//
//    fun goToBasketRCurve(): BezierCurve {
//        return BezierCurve(
//            PoseStorage.currentPoint,
//            Point(108.0, 24.0),
//            redBasket
//        )
//    }
//
//    fun goToNeutralSampleBCurve(): BezierCurve {
//        return BezierCurve(
//            PoseStorage.currentPoint,
//            Point(30.0, 120.0),
//            blueNeutralSample
//        )
//    }
//
//    fun goToNeutralSampleRCurve(): BezierCurve {
//        return BezierCurve(
//            PoseStorage.currentPoint,
//            Point(114.0, 24.0),
//            redNeutralSample
//        )
//    }
//
//    fun goToSampleBCurve(offsetY: Double): BezierCurve {
//        return BezierCurve(
//            PoseStorage.currentPoint,
//            Point(64.0, 22.0),
//            Point(blueSample.x, blueSample.y + offsetY)
//        )
//    }
//
//    fun goToSampleRCurve(offsetY: Double): BezierCurve {
//        return BezierCurve(
//            PoseStorage.currentPoint,
//            Point(84.0, 132.0),
//            Point(redSample.x, redSample.y + offsetY)
//        )
//    }
//
//    fun goToEndBlCurve(): BezierCurve {
//        return BezierCurve(
//            PoseStorage.currentPoint,
//            Point(26.0, 120.0),
//            Point(64.0, 114.0),
//            blueEndLeft
//        )
//    }
//
//    fun goToEndBrCurve(): BezierCurve {
//        return BezierCurve(
//            PoseStorage.currentPoint,
//            Point(26.0, 120.0),
//            Point(64.0, 114.0),
//            blueEndRight
//        )
//    }
//
//    fun goToEndRlCurve(): BezierCurve {
//        return BezierCurve(
//            PoseStorage.currentPoint,
//            Point(118.0, 24.0),
//            Point(80.0, 30.0),
//            redEndLeft
//        )
//    }
//
//    fun goToHumanBCurve(): BezierCurve {
//        return BezierCurve(
//            PoseStorage.currentPoint,
//            Point(34.0, 34.0),
//            blueHuman
//        )
//    }

    enum class statesRR {
        DRIVE_TO_SPECIMEN_R,
        PLACE_SPECIMEN_R,
        GO_TO_SAMPLE_R,
        GO_TO_SAMPLE_R2,
        GO_TO_SAMPLE_R3,
        GO_TO_HUMAN_R,
        GO_TO_SPECIMEN_R,
        PLACE_SPECIMEN_R1,
        GO_TO_HUMAN_R2,
        GO_TO_SPECIMEN_R2,
        PLACE_SPECIMEN_R2,
        GO_TO_HUMAN_R3,
        GO_TO_SPECIMEN_R3,
        PLACE_SPECIMEN_R3,
        GO_TO_END_R,
        STOP,
    }

    fun scorePreload() {
        runBlocking(
            ParallelAction(
                drive.actionBuilder(redStartRight)
                    .splineToConstantHeading(
                        Vector2d(redSpecimen.position.x, redSpecimen.position.y),
                        redSpecimen.heading.toDouble()
                    )
                    .build(),
                driverAid.daAction(listOf(Runnable { driverAid.highSpecimen() })),
            ),

            )
        armSubsystem.eMax = 0.5
        runBlocking(
            SequentialAction(
                armSubsystem.setPEAction(hSpecimenP, 600.0),
                scoringSubsystem.servoAction(
                    listOf(
                        Runnable { scoringSubsystem.openClaw() },
                        Runnable { scoringSubsystem.setRotateCenter() }
                    )
                )
            )
        )
        armSubsystem.eMax = 1.0


    }

    fun moveAll() {
        runBlocking(
            ParallelAction(
                drive.actionBuilder(redSpecimen)
                    .setTangent(Math.toRadians(-90.0))
                    .splineToLinearHeading(
                        Pose2d(36.0, -24.0, redSample.heading.toDouble()),
                        redSpecimen.heading.toDouble()
                    )
                    .splineToConstantHeading(
                        Vector2d(redSample.position.x - 12.0, redSample.position.y),
                        redSample.heading.toDouble()
                    )
                    .setTangent(redSpecimen.heading.toDouble())
                    .lineToY(redHuman.position.y)
                    .setTangent(redStartLeft.heading.toDouble())
                    .splineToLinearHeading(
                        Pose2d(
                            redSample.position.x - 3,
                            redSample.position.y,
                            redSample.heading.toDouble()
                        ), redSample.heading.toDouble()
                    )
                    .strafeTo(Vector2d(redSample.position.x - 3, redHuman.position.y))
                    .setTangent(redStartRight.heading.toDouble())
                    .splineToConstantHeading(
                        Vector2d(
                            redSample.position.x + 1,
                            redSample.position.y
                        ), redSample.heading.toDouble()
                    )
                    .strafeTo(Vector2d(redSample.position.x + 1, redHuman.position.y))

                    .build(),

                driverAid.daAction(listOf(Runnable { driverAid.collapse() }))
            )
        )

    }

    fun grab1() {
        runBlocking(
            ParallelAction(
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
                driverAid.daAction(listOf(Runnable { driverAid.human() }))
            )
        )
    }

    fun place() {
        runBlocking(
            ParallelAction(
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
                driverAid.daAction(listOf(Runnable { driverAid.highSpecimen() }))
            )
        )
    }

    fun end() {
        runBlocking(
            ParallelAction(
                drive.actionBuilder(
                    redSpecimen
                )
                    .strafeTo(Vector2d(36.0, -31.0))
                    .setTangent(Math.toRadians(90.0))
                    .splineToLinearHeading(redEndRight, blueEndRight.heading.toDouble())

                    .build(),
                driverAid.daAction(listOf(Runnable { driverAid.collapse() }))
            )
        )
    }
//TODO this is code for end

//                .strafeTo(new Vector2d(36, -31))
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(redEndRight, blueEndRight.heading.toDouble())

//    val smallSpecimenAutoR: StateMachine<statesRR> = StateMachine.Builder<statesRR>()
//        .state(statesRR.DRIVE_TO_SPECIMEN_R)
//        .onEnter(statesRR.DRIVE_TO_SPECIMEN_R) {
//        }
//        .whileState(statesRR.DRIVE_TO_SPECIMEN_R, { drive. }) {
//            updateAll()
//            telemetry.addData("x", follower.pose.x);
//            telemetry.addData("y", follower.pose.y);
//            telemetry.addData("heading", follower.pose.heading);
//            telemetry.update();
//        }
//        .transition(statesRR.DRIVE_TO_SPECIMEN_R, { follower.atParametricEnd() }, 0.0)
//        .state(statesRR.PLACE_SPECIMEN_R)
//        .onEnter(statesRR.PLACE_SPECIMEN_R) {
//            armSubsystem.eMax = 0.5
//            armSubsystem.setExtendTarget(600.0)
//        }
//        .whileState(statesRR.PLACE_SPECIMEN_R, {
//            armSubsystem.isExtendAtTarget(30.0)
//        }) {
//            armSubsystem.update()
//        }
//        .onExit(statesRR.PLACE_SPECIMEN_R) {
//            armSubsystem.eMax = 1.0
//            scoringSubsystem.openClaw()
//            scoringSubsystem.update()
//        }
//        .transition(
//            statesRR.PLACE_SPECIMEN_R,
//            {
//                (true)
//            },
//            0.0
//        )
//        .state(statesRR.GO_TO_END_R)
//        .onEnter(statesRR.GO_TO_END_R) {
//            driverAid.human()
//            follower.followPath(move1)
//        }
//        .whileState(statesRR.GO_TO_END_R, { follower.atParametricEnd() }) {
//            updateAll()
//            telemetry.addData("x", follower.pose.x);
//            telemetry.addData("y", follower.pose.y);
//            telemetry.addData("heading", follower.pose.heading);
//            telemetry.update();
//        }
//        .transition(statesRR.GO_TO_END_R, { true }, 0.0)
//        .stopRunning(statesRR.STOP)
//        .build()
}