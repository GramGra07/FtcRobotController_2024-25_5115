package org.firstinspires.ftc.teamcode.customHardware

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.followers.pedroPathing.follower.Follower
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.BezierCurve
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
    val redHumanAngle = Math.toRadians(180.0)
    val blueBasket = Point(22.0, 122.0)
    val blueBasketAngle = Math.toRadians(45.0)
    val redBasket = Point(122.0, 22.0)
    val redBasketAngle = Math.toRadians(135.0)
    val blueSpecimen = Point(36.0, 72.0)
    val blueSpecimenAngle = Math.toRadians(180.0)
    val redSpecimen = Point(108.0, 72.0)
    val redSpecimenAngle = Math.toRadians(0.0)
    val blueSample = Point(60.0, 12.0)
    val blueSampleAngle = Math.toRadians(-90.0)
    val redSample = Point(84.0, 132.0)
    val redSampleAngle = Math.toRadians(-90.0)
    val blueNeutralSample = Point(26.0, 132.0)
    val redNeutralSample = Point(118.0, 12.0)

    fun goToSpecimenBCurve(): BezierCurve {
        return BezierCurve(
            PoseStorage.currentPoint,
            Point(24.0, 72.0),
            blueSpecimen
        )
    }

    fun goToSpecimenRCurve(): BezierCurve {
        return BezierCurve(
            PoseStorage.currentPoint,
            Point(126.0, 72.0),
            redSpecimen
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

    fun goToEndRrCurve(): BezierCurve {
        return BezierCurve(
            PoseStorage.currentPoint,
            Point(118.0, 24.0),
            Point(80.0, 30.0),
            redEndRight
        )
    }

    fun goToHumanBCurve(): BezierCurve {
        return BezierCurve(
            PoseStorage.currentPoint,
            Point(34.0, 34.0),
            blueHuman
        )
    }

    fun goToHumanRCurve(): BezierCurve {
        return BezierCurve(
            PoseStorage.currentPoint,
            Point(108.0, 108.0),
            redHuman
        )
    }

    val pathBuilder = this.localizerSubsystem.follower.pathBuilder()

    fun placeSpecimenBPath(): PathChain {
        return pathBuilder.addPath(
            goToSpecimenBCurve()
        ).setLinearHeadingInterpolation(PoseStorage.currentHeading, blueSpecimenAngle)
            .build()
    }

    fun placeSpecimenRPath(): PathChain {
        return pathBuilder.addPath(
            goToSpecimenRCurve()
        ).setConstantHeadingInterpolation(redSpecimenAngle)
            .build()
    }

    fun placeBasketBPath(): PathChain {
        return pathBuilder.addPath(
            goToBasketBCurve()
        ).setLinearHeadingInterpolation(PoseStorage.currentHeading, blueBasketAngle)
            .build()
    }

    fun placeBasketRPath(): PathChain {
        return pathBuilder.addPath(
            goToBasketRCurve()
        ).setLinearHeadingInterpolation(PoseStorage.currentHeading, redBasketAngle)
            .build()
    }

    fun getSpecimenRPath(): PathChain {
        return pathBuilder.addPath(
            goToHumanRCurve()
        ).setLinearHeadingInterpolation(PoseStorage.currentHeading, redHumanAngle)
            .build()
    }

    fun getSpecimenBPath(): PathChain {
        return pathBuilder.addPath(
            goToHumanBCurve()
        ).setLinearHeadingInterpolation(PoseStorage.currentHeading, blueHumanAngle)
            .build()
    }

    fun getSampleRPath(offsetY: Double): PathChain {
        return pathBuilder.addPath(
            goToSampleRCurve(offsetY)
        ).setLinearHeadingInterpolation(PoseStorage.currentHeading, redSampleAngle)
            .build()
    }

    fun getSampleBPath(offsetY: Double): PathChain {
        return pathBuilder.addPath(
            goToSampleBCurve(offsetY)
        ).setLinearHeadingInterpolation(PoseStorage.currentHeading, blueSampleAngle)
            .build()
    }

    fun goToEndBlPath(): PathChain {
        return pathBuilder.addPath(
            goToEndBlCurve()
        ).setLinearHeadingInterpolation(PoseStorage.currentHeading, blueEndAngle)
            .build()
    }

    fun goToEndBrPath(): PathChain {
        return pathBuilder.addPath(
            goToEndBrCurve()
        ).setLinearHeadingInterpolation(PoseStorage.currentHeading, blueEndAngle)
            .build()
    }

    fun goToEndRlPath(): PathChain {
        return pathBuilder.addPath(
            goToEndRlCurve()
        ).setLinearHeadingInterpolation(PoseStorage.currentHeading, redEndAngle)
            .build()
    }

    fun goToEndRrPath(): PathChain {
        return pathBuilder.addPath(
            goToEndRrCurve()
        ).setLinearHeadingInterpolation(PoseStorage.currentHeading, redEndAngle)
            .build()
    }

    fun goToNeutralSampleB(angle: Double): PathChain {
        return pathBuilder.addPath(
            goToNeutralSampleBCurve()
        ).setLinearHeadingInterpolation(PoseStorage.currentHeading, angle)
            .build()
    }

    fun goToNeutralSampleR(angle: Double): PathChain {
        return pathBuilder.addPath(
            goToNeutralSampleRCurve()
        ).setLinearHeadingInterpolation(PoseStorage.currentHeading, angle)
            .build()
    }

    private fun refreshPose() {
        PoseStorage.currentPose = follower.pose
    }

    fun placeSpecimenBPathFollower() {
        follower.followPath(placeSpecimenBPath())
        refreshPose()
    }

    fun placeSpecimenRPathFollower() {
        follower.followPath(placeSpecimenRPath())
    }

    fun goToBasketBPathFollower() {
        follower.followPath(placeBasketBPath())
        refreshPose()
    }

    fun goToBasketRPathFollower() {
        follower.followPath(placeBasketRPath())
        refreshPose()
    }

    fun goToSampleBPathFollower(offsetY: Double) {
        follower.followPath(getSampleBPath(offsetY))
        refreshPose()
    }

    fun goToSampleRPathFollower(offsetY: Double) {
        follower.followPath(getSampleRPath(offsetY))
        refreshPose()
    }

    fun goToEndBlPathFollower() {
        follower.followPath(goToEndBlPath())
        refreshPose()
    }

    fun goToEndBrPathFollower() {
        follower.followPath(goToEndBrPath())
        refreshPose()
    }

    fun goToEndRlPathFollower() {
        follower.followPath(goToEndRlPath())
        refreshPose()
    }

    fun goToEndRrPathFollower() {
        follower.followPath(goToEndRrPath())
        refreshPose()
    }

    fun goToHumanBPathFollower() {
        follower.followPath(getSpecimenBPath())
        refreshPose()
    }

    fun goToHumanRPathFollower() {
        follower.followPath(getSpecimenRPath())
        refreshPose()
    }

    fun goToNeutralSampleBPathFollower(angle: Double) {
        follower.followPath(goToNeutralSampleB(angle))
        refreshPose()
    }

    fun goToNeutralSampleRPathFollower(angle: Double) {
        follower.followPath(goToNeutralSampleR(angle))
        refreshPose()
    }

    enum class statesRR {
        driveToSpecimenR,
        placeSpecimenR,
        goToSampleR,
        goToSampleR2,
        goToSampleR3,
        goToHumanR,
        goToSpecimenR,
        placeSpecimenR1,
        goToHumanR2,
        goToSpecimenR2,
        placeSpecimenR2,
        goToHumanR3,
        goToSpecimenR3,
        placeSpecimenR3,
        goToEndR,
        stop,
    }

    val specimenAutoR: StateMachine<statesRR> = StateMachine.Builder<statesRR>()
        .state(statesRR.driveToSpecimenR)
        .onEnter(statesRR.driveToSpecimenR) {
            driverAid.highSpecimen()
            placeSpecimenRPathFollower()
        }
        .whileState(statesRR.driveToSpecimenR, { follower.atParametricEnd() }) {
            updateAll()
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
        .transition(statesRR.driveToSpecimenR, { follower.atParametricEnd() }, 0.5)
        .state(statesRR.placeSpecimenR)
        .onEnter(statesRR.placeSpecimenR) {
            driverAid.collapse()
            driverAid.update()
        }
        .whileState(statesRR.placeSpecimenR, {
            armSubsystem.bothAtTarget(30.0)
        }) {
            updateAll()
        }
        .transition(
            statesRR.placeSpecimenR,
            {
                (true)
            },
            0.0
        )
        .stopRunning(statesRR.stop)
        .build()
}