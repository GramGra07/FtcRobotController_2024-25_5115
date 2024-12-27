package org.firstinspires.ftc.teamcode.customHardware

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.followers.pedroPathing.follower.Follower
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.BezierCurve
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.BezierLine
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.PathChain
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.Point
import org.firstinspires.ftc.teamcode.utilClass.storage.PoseStorage
import org.gentrifiedApps.statemachineftc.StateMachine


class AutoHardware(
    opmode: LinearOpMode,
    startLocation: StartLocation,
    ahwMap: HardwareMap = opmode.hardwareMap,
) : HardwareConfig(opmode, true, startLocation, ahwMap) {
    lateinit var follower: Follower

    init {
        super.initRobot(ahwMap, true, startLocation)
        follower = super.localizerSubsystem.follower
        buildPaths()
        telemetry.addData("Status", "Initialized")
        telemetry.addData("Alliance", startLocation.alliance)
        telemetry.update()
        startLocation.build()
        opmode.waitForStart()
        timer.reset()
    }

    fun updateAll() {
        follower.update()
        driverAid.update()
        armSubsystem.update()
        scoringSubsystem.update()
    }

    fun autoSetup() {
        this.scoringSubsystem.setup()
    }

    fun pause(time: Double, runWhilePaused: () -> Unit) {
        val myTimer = ElapsedTime()
        myTimer.reset()
        while (myTimer.seconds() < time) {
            telemetry.addData("Elapsed Time", myTimer.seconds())
            telemetry.update()
            runWhilePaused()
            follower.holdPoint(follower.pose)
        }
    }


    val blueX = 6.0
    val blueStartLeft = Point(blueX, 108.0)
    val blueStartRight = Point(blueX, 60.0)
    val blueStartAngle = Math.toRadians(180.0)
    val redX = 136.0
    val redStartLeft = Point(redX, 88.0)
    val redStartRight = Point(redX, 36.0)
    val redStartAngle = Math.toRadians(0.0)

    val blueEndLeft = Point(96.0, 62.0)
    val blueEndRight = Point(62.0, 108.0)
    val blueEndAngle = Math.toRadians(-90.0)
    val redEndLeft = Point(62.0, 48.0)
    val redEndRight = Point(82.0, 48.0)
    val redEndAngle = Math.toRadians(90.0)
    val blueHuman = Point(12.0, 24.0)
    val blueHumanAngle = Math.toRadians(0.0)
    val redHuman = Point(130.0, 120.0)
    val redHumanAngle = Math.toRadians(0.0)
    val blueBasket = Point(22.0, 122.0)
    val blueBasketAngle = Math.toRadians(45.0)
    val redBasket = Point(122.0, 22.0)
    val redBasketAngle = Math.toRadians(135.0)
    val blueSpecimen = Point(36.0, 72.0)
    val blueSpecimenAngle = Math.toRadians(180.0)
    val redSpecimen = Point(102.5, 72.0)
    val redSpecimenAngle = Math.toRadians(0.0)
    val blueSample = Point(60.0, 12.0)
    val blueSampleAngle = Math.toRadians(-90.0)
    val redSample = Point(84.0, 132.0)
    val redSampleAngle = Math.toRadians(-90.0)
    val blueNeutralSample = Point(26.0, 132.0)
    val redNeutralSample = Point(118.0, 12.0)

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
        scorePreload = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    PoseStorage.currentPoint,
                    Point(126.0, 72.0),
                    redSpecimen
                )
            )
            .setLinearHeadingInterpolation(redStartAngle, redSpecimenAngle)
            .build()

        move1 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    redSpecimen,
                    Point(84.0, 132.0),
                    redSample
                )
            )
            .setLinearHeadingInterpolation(redSpecimenAngle, redSampleAngle)
            .addPath(BezierLine(redSample, redHuman))
            .setConstantHeadingInterpolation(redSampleAngle)
            .build()
        move2 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    redHuman,
                    Point(84.0, 132.0),
                    redSample
                )
            )
            .setLinearHeadingInterpolation(redSampleAngle, redHumanAngle)
            .addPath(BezierLine(redSample, redSpecimen))
            .setConstantHeadingInterpolation(redSampleAngle)
            .build()
        move3 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    redSpecimen,
                    Point(84.0, 132.0),
                    redSample
                )
            )
            .setLinearHeadingInterpolation(redSpecimenAngle, redSampleAngle)
            .addPath(BezierLine(redSample, redHuman))
            .setConstantHeadingInterpolation(redSampleAngle)
            .build()
        pickup1 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    redHuman,
                    redHuman
                )
            )
            .setConstantHeadingInterpolation(redHumanAngle)
            .build()
        score1 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    redHuman,
                    Point(126.0, 72.0),
                    redSpecimen
                )
            )
            .setLinearHeadingInterpolation(redHumanAngle, redSpecimenAngle)
            .build()
        pickup2 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    redSpecimen,
                    Point(126.0, 72.0),
                    redHuman
                )
            )
            .setLinearHeadingInterpolation(redSpecimenAngle, redHumanAngle)
            .build()
        score2 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    redHuman,
                    Point(126.0, 72.0),
                    redSpecimen
                )
            )
            .setLinearHeadingInterpolation(redHumanAngle, redSpecimenAngle)
            .build()
        pickup3 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    redSpecimen,
                    Point(126.0, 72.0),
                    redHuman
                )
            )
            .setLinearHeadingInterpolation(redSpecimenAngle, redHumanAngle)
            .build()
        score3 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    redHuman,
                    Point(126.0, 72.0),
                    redSpecimen
                )
            )
            .setLinearHeadingInterpolation(redHumanAngle, redSpecimenAngle)
            .build()
        end = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    redSpecimen,
                    Point(118.0, 24.0),
                    Point(80.0, 30.0),
                    redEndRight
                )
            )
            .setLinearHeadingInterpolation(redSpecimenAngle, redEndAngle)
            .build()
    }

    fun goToSpecimenBCurve(): BezierCurve {
        return BezierCurve(
            PoseStorage.currentPoint,
            Point(24.0, 72.0),
            blueSpecimen
        )
    }

    fun goToBasketBCurve(): BezierCurve {
        return BezierCurve(
            PoseStorage.currentPoint,
            Point(36.0, 120.0),
            blueBasket
        )
    }

    fun goToBasketRCurve(): BezierCurve {
        return BezierCurve(
            PoseStorage.currentPoint,
            Point(108.0, 24.0),
            redBasket
        )
    }

    fun goToNeutralSampleBCurve(): BezierCurve {
        return BezierCurve(
            PoseStorage.currentPoint,
            Point(30.0, 120.0),
            blueNeutralSample
        )
    }

    fun goToNeutralSampleRCurve(): BezierCurve {
        return BezierCurve(
            PoseStorage.currentPoint,
            Point(114.0, 24.0),
            redNeutralSample
        )
    }

    fun goToSampleBCurve(offsetY: Double): BezierCurve {
        return BezierCurve(
            PoseStorage.currentPoint,
            Point(64.0, 22.0),
            Point(blueSample.x, blueSample.y + offsetY)
        )
    }

    fun goToSampleRCurve(offsetY: Double): BezierCurve {
        return BezierCurve(
            PoseStorage.currentPoint,
            Point(84.0, 132.0),
            Point(redSample.x, redSample.y + offsetY)
        )
    }

    fun goToEndBlCurve(): BezierCurve {
        return BezierCurve(
            PoseStorage.currentPoint,
            Point(26.0, 120.0),
            Point(64.0, 114.0),
            blueEndLeft
        )
    }

    fun goToEndBrCurve(): BezierCurve {
        return BezierCurve(
            PoseStorage.currentPoint,
            Point(26.0, 120.0),
            Point(64.0, 114.0),
            blueEndRight
        )
    }

    fun goToEndRlCurve(): BezierCurve {
        return BezierCurve(
            PoseStorage.currentPoint,
            Point(118.0, 24.0),
            Point(80.0, 30.0),
            redEndLeft
        )
    }

    fun goToHumanBCurve(): BezierCurve {
        return BezierCurve(
            PoseStorage.currentPoint,
            Point(34.0, 34.0),
            blueHuman
        )
    }

    private fun refreshPose() {
        PoseStorage.currentPose = follower.pose
    }

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

    val smallSpecimenAutoR: StateMachine<statesRR> = StateMachine.Builder<statesRR>()
        .state(statesRR.DRIVE_TO_SPECIMEN_R)
        .onEnter(statesRR.DRIVE_TO_SPECIMEN_R) {
            armSubsystem.setExtendTarget(1100.0)
            armSubsystem.setPitchTargetDegrees(65.0)
            scoringSubsystem.setPitchLow()
            scoringSubsystem.setRotateCenter()
            follower.followPath(scorePreload, true)
        }
        .whileState(statesRR.DRIVE_TO_SPECIMEN_R, { follower.atParametricEnd() }) {
            updateAll()
            telemetry.addData("x", follower.pose.x);
            telemetry.addData("y", follower.pose.y);
            telemetry.addData("heading", follower.pose.heading);
            telemetry.update();
        }
        .transition(statesRR.DRIVE_TO_SPECIMEN_R, { follower.atParametricEnd() }, 0.0)
        .state(statesRR.PLACE_SPECIMEN_R)
        .onEnter(statesRR.PLACE_SPECIMEN_R) {
            armSubsystem.eMax = 0.5
            armSubsystem.setExtendTarget(600.0)
        }
        .whileState(statesRR.PLACE_SPECIMEN_R, {
            armSubsystem.isExtendAtTarget(30.0)
        }) {
            armSubsystem.update()
        }
        .onExit(statesRR.PLACE_SPECIMEN_R) {
            armSubsystem.eMax = 1.0
            scoringSubsystem.openClaw()
            scoringSubsystem.update()
        }
        .transition(
            statesRR.PLACE_SPECIMEN_R,
            {
                (true)
            },
            0.0
        )
        .state(statesRR.GO_TO_END_R)
        .onEnter(statesRR.GO_TO_END_R) {
            driverAid.human()
            follower.followPath(move1)
        }
        .whileState(statesRR.GO_TO_END_R, { follower.atParametricEnd() }) {
            updateAll()
            telemetry.addData("x", follower.pose.x);
            telemetry.addData("y", follower.pose.y);
            telemetry.addData("heading", follower.pose.heading);
            telemetry.update();
        }
        .transition(statesRR.GO_TO_END_R, { true }, 0.0)
        .stopRunning(statesRR.STOP)
        .build()
}