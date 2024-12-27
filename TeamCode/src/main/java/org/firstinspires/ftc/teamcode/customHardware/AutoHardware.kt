package org.firstinspires.ftc.teamcode.customHardware

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.PathChain
import org.firstinspires.ftc.teamcode.followers.roadRunner.MecanumDrive


class AutoHardware(
    opmode: LinearOpMode,
    startLocation: StartLocation,
    ahwMap: HardwareMap = opmode.hardwareMap,
) : HardwareConfig(opmode, true, startLocation, ahwMap) {
    lateinit var drive: MecanumDrive

    init {
        super.initRobot(ahwMap, true, startLocation)
        drive = super.localizerSubsystem.drive
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
        val blueStartLeft = Pose2d(6.0 - 72, 108.0 - 72, Math.toRadians(180.0))
        val blueStartRight = Pose2d(6.0 - 72, 60.0 - 72, Math.toRadians(180.0))
        val redStartLeft = Pose2d(135.0 - 72, 88.0 - 72, Math.toRadians(0.0))
        val redStartRight = Pose2d(135.0 - 72, 80.0 - 72, Math.toRadians(0.0))
        val blueEndLeft = Pose2d(96.0 - 72, 62.0 - 72, Math.toRadians(-90.0))
        val blueEndRight = Pose2d(62.0 - 72, 108.0 - 72, Math.toRadians(-90.0))
        val redEndLeft = Pose2d(62.0 - 72, 48.0 - 72, Math.toRadians(90.0))
        val redEndRight = Pose2d(82.0 - 72, 48.0 - 72, Math.toRadians(90.0))
        val blueHuman = Pose2d(12.0 - 72, 24.0 - 72, Math.toRadians(0.0))
        val redHuman = Pose2d(130.0 - 72, 120.0 - 72, Math.toRadians(0.0))
        val blueBasket = Pose2d(22.0 - 72, 122.0 - 72, Math.toRadians(45.0))
        val redBasket = Pose2d(122.0 - 72, 22.0 - 72, Math.toRadians(135.0))
        val blueSpecimen = Pose2d(36.0 - 72, 72.0 - 72, Math.toRadians(180.0))
        val redSpecimen = Pose2d(102.5 - 72, 72.0 - 72, Math.toRadians(0.0))
        val blueSample = Pose2d(60.0 - 72, 12.0 - 72, Math.toRadians(-90.0))
        val redSample = Pose2d(84.0 - 72, 132.0 - 72, Math.toRadians(-90.0))
        val blueNeutralSample = Pose2d(26.0 - 72, 132.0 - 72, Math.toRadians(0.0))
        val redNeutralSample = Pose2d(118.0 - 72, 12.0 - 72, Math.toRadians(0.0))
    }


    // for specimen auto
    lateinit var scorePreload: PathChain
    lateinit var move1: PathChain
    lateinit var move2: PathChain
    lateinit var move3: PathChain
    lateinit var pickup1: PathChain
    lateinit var score1: PathChain
    lateinit var pickup2: PathChain
    lateinit var score2: PathChain
    lateinit var pickup3: PathChain
    lateinit var score3: PathChain
    lateinit var end: PathChain

    fun buildPaths() {
//        scorePreload = follower.pathBuilder()
//            .addPath(
//                BezierCurve(
//                    Point(redStartRight),
//                    Point(126.0, 72.0),
//                    Point(redSpecimen)
//                )
//            )
//            .setLinearHeadingInterpolation(redStartRight.heading, redSpecimen.heading)
//            .build()
//
//        move1 = follower.pathBuilder()
//            .addPath(
//                BezierCurve(
//                    Point(redSpecimen),
//                    Point(84.0, 132.0),
//                    Point(redSample)
//                )
//            )
//            .setLinearHeadingInterpolation(redSpecimen.heading, redSample.heading)
//            .addPath(BezierLine(Point(redSample), Point(redHuman)))
//            .setConstantHeadingInterpolation(redSample.heading)
//            .build()
//
//        move2 = follower.pathBuilder()
//            .addPath(
//                BezierCurve(
//                    Point(redHuman),
//                    Point(84.0, 132.0),
//                    Point(redSample)
//                )
//            )
//            .setLinearHeadingInterpolation(redHuman.heading, redSample.heading)
//            .addPath(BezierLine(Point(redSample), Point(redSpecimen)))
//            .setConstantHeadingInterpolation(redSample.heading)
//            .build()
//
//        move3 = follower.pathBuilder()
//            .addPath(
//                BezierCurve(
//                    Point(redSpecimen),
//                    Point(84.0, 132.0),
//                    Point(redSample)
//                )
//            )
//            .setLinearHeadingInterpolation(redSpecimen.heading, redSample.heading)
//            .addPath(BezierLine(Point(redSample), Point(redHuman)))
//            .setConstantHeadingInterpolation(redSample.heading)
//            .build()
//
//        pickup1 = follower.pathBuilder()
//            .addPath(
//                BezierCurve(
//                    Point(redHuman),
//                    Point(redHuman)
//                )
//            )
//            .setConstantHeadingInterpolation(redHuman.heading)
//            .build()
//
//        score1 = follower.pathBuilder()
//            .addPath(
//                BezierCurve(
//                    Point(redHuman),
//                    Point(126.0, 72.0),
//                    Point(redSpecimen)
//                )
//            )
//            .setLinearHeadingInterpolation(redHuman.heading, redSpecimen.heading)
//            .build()
//
//        pickup2 = follower.pathBuilder()
//            .addPath(
//                BezierCurve(
//                    Point(redSpecimen),
//                    Point(126.0, 72.0),
//                    Point(redHuman)
//                )
//            )
//            .setLinearHeadingInterpolation(redSpecimen.heading, redHuman.heading)
//            .build()
//
//        score2 = follower.pathBuilder()
//            .addPath(
//                BezierCurve(
//                    Point(redHuman),
//                    Point(126.0, 72.0),
//                    Point(redSpecimen)
//                )
//            )
//            .setLinearHeadingInterpolation(redHuman.heading, redSpecimen.heading)
//            .build()
//
//        pickup3 = follower.pathBuilder()
//            .addPath(
//                BezierCurve(
//                    Point(redSpecimen),
//                    Point(126.0, 72.0),
//                    Point(redHuman)
//                )
//            )
//            .setLinearHeadingInterpolation(redSpecimen.heading, redHuman.heading)
//            .build()
//
//        score3 = follower.pathBuilder()
//            .addPath(
//                BezierCurve(
//                    Point(redHuman),
//                    Point(126.0, 72.0),
//                    Point(redSpecimen)
//                )
//            )
//            .setLinearHeadingInterpolation(redHuman.heading, redSpecimen.heading)
//            .build()
//
//        end = follower.pathBuilder()
//            .addPath(
//                BezierCurve(
//                    Point(redSpecimen),
//                    Point(118.0, 24.0),
//                    Point(80.0, 30.0),
//                    Point(redEndRight)
//                )
//            )
//            .setLinearHeadingInterpolation(redSpecimen.heading, redEndRight.heading)
//            .build()
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

    fun smallSpecimenAuto() {
        scoringSubsystem.setPitchLow()
        scoringSubsystem.setRotateCenter()
        runBlocking(
            drive.actionBuilder(redStartRight)
                .splineTo(Vector2d(126.0 - 72, 72.0 - 72), redStartRight.heading.toDouble())
                .splineTo(
                    Vector2d(redSpecimen.position.x, redSpecimen.position.y),
                    redSpecimen.heading.toDouble()
                )
                .build()
        )
        armSubsystem.setExtendTarget(1100.0)
        armSubsystem.setPitchTargetDegrees(65.0)
        //wait

        armSubsystem.eMax = 0.5
        armSubsystem.setExtendTarget(600.0)
        //wait
        armSubsystem.eMax = 1.0
        scoringSubsystem.openClaw()
        scoringSubsystem.update()
//        runBlocking(
//            drive.actionBuilder(redSpecimen)
//                .splineTo(Vector2d(84.0, 132.0), redSpecimen.heading.toDouble())
//                .splineTo(Vector2d(redSample.position.x, redSample.position.y), redSample.heading.toDouble())
//                .build()
//        )
        // go to end
        driverAid.human()
        runBlocking(
            drive.actionBuilder(redSpecimen)
                .splineTo(
                    Vector2d(redHuman.position.x, redHuman.position.y),
                    redSpecimen.heading.toDouble()
                )
                .build()
        )

    }

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