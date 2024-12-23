package org.firstinspires.ftc.teamcode.customHardware

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
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
            driverAid.highSpecimen()
            placeSpecimenRPathFollower()
        }
        .whileState(statesRR.DRIVE_TO_SPECIMEN_R, { follower.atParametricEnd() }) {
            updateAll()
            telemetry.addData("x", follower.pose.x);
            telemetry.addData("y", follower.pose.y);
            telemetry.addData("heading", follower.pose.heading);
            telemetry.update();
        }
        .transition(statesRR.DRIVE_TO_SPECIMEN_R, { follower.atParametricEnd() }, 0.5)
        .state(statesRR.PLACE_SPECIMEN_R)
        .onEnter(statesRR.PLACE_SPECIMEN_R) {
            armSubsystem.setExtendTarget(900.0)
        }
        .whileState(statesRR.PLACE_SPECIMEN_R, {
            armSubsystem.isExtendAtTarget(30.0)
        }) {
            updateAll()
        }
        .transition(
            statesRR.PLACE_SPECIMEN_R,
            {
                (true)
            },
            0.0
        )
        .stopRunning(statesRR.STOP)
        .build()

    val fullSpecimenAutoR: StateMachine<statesRR> = StateMachine.Builder<statesRR>()
        .state(statesRR.DRIVE_TO_SPECIMEN_R)
        .onEnter(statesRR.DRIVE_TO_SPECIMEN_R) {
            driverAid.highSpecimen()
            placeSpecimenRPathFollower()
        }
        .whileState(statesRR.DRIVE_TO_SPECIMEN_R, { follower.atParametricEnd() }) {
            updateAll()
            telemetry.addData("x", follower.pose.x);
            telemetry.addData("y", follower.pose.y);
            telemetry.addData("heading", follower.pose.heading);
            telemetry.update();
        }
        .transition(statesRR.DRIVE_TO_SPECIMEN_R, { follower.atParametricEnd() }, 0.5)
        .state(statesRR.PLACE_SPECIMEN_R)
        .onEnter(statesRR.PLACE_SPECIMEN_R) {
            armSubsystem.setExtendTarget(900.0)
        }
        .whileState(statesRR.PLACE_SPECIMEN_R, {
            armSubsystem.isExtendAtTarget(30.0)
        }) {
            updateAll()
        }
        .transition(
            statesRR.PLACE_SPECIMEN_R,
            {
                (true)
            },
            0.0
        )
        .state(statesRR.GO_TO_SAMPLE_R)
        .onEnter(statesRR.GO_TO_SAMPLE_R) {
            goToSampleRCurve(0.0)
        }
        .whileState(statesRR.GO_TO_SAMPLE_R, { follower.atParametricEnd() }) {
            updateAll()
        }
        .transition(statesRR.GO_TO_SAMPLE_R, { follower.atParametricEnd() }, 0.5)
        .state(statesRR.GO_TO_SAMPLE_R2)
        .onEnter(statesRR.GO_TO_SAMPLE_R2) {
            goToSampleRCurve(0.0)
        }
        .whileState(statesRR.GO_TO_SAMPLE_R2, { follower.atParametricEnd() }) {
            updateAll()
        }
        .transition(statesRR.GO_TO_SAMPLE_R2, { follower.atParametricEnd() }, 0.5)

        .state(statesRR.GO_TO_SAMPLE_R3)
        .onEnter(statesRR.GO_TO_SAMPLE_R3) {
            goToSampleRCurve(0.0)
        }
        .whileState(statesRR.GO_TO_SAMPLE_R3, { follower.atParametricEnd() }) {
            updateAll()
        }
        .transition(statesRR.GO_TO_SAMPLE_R3, { follower.atParametricEnd() }, 0.5)
        .state(statesRR.GO_TO_HUMAN_R)
        .onEnter(statesRR.GO_TO_HUMAN_R) {
            driverAid.human()
            driverAid.update()
            goToHumanRPathFollower()
        }
        .whileState(statesRR.GO_TO_HUMAN_R, { follower.atParametricEnd() }) {
            updateAll()
        }
        .transition(statesRR.GO_TO_HUMAN_R, { follower.atParametricEnd() }, 0.5)
        .state(statesRR.GO_TO_SPECIMEN_R)
        .onEnter(statesRR.GO_TO_SPECIMEN_R) {
            goToSpecimenRCurve()
            driverAid.highSpecimen()
            driverAid.update()
        }
        .whileState(statesRR.GO_TO_SPECIMEN_R, { follower.atParametricEnd() }) {
            updateAll()
        }
        .transition(statesRR.GO_TO_SPECIMEN_R, { follower.atParametricEnd() }, 0.5)
        .state(statesRR.PLACE_SPECIMEN_R1)
        .onEnter(statesRR.PLACE_SPECIMEN_R1) {
            armSubsystem.setExtendTarget(900.0)
        }
        .whileState(statesRR.PLACE_SPECIMEN_R1, {
            armSubsystem.isExtendAtTarget(30.0)
        }) {
            updateAll()
        }
        .transition(
            statesRR.PLACE_SPECIMEN_R1,
            {
                (true)
            },
            0.0
        )
        .state(statesRR.GO_TO_HUMAN_R2)
        .onEnter(statesRR.GO_TO_HUMAN_R2) {
            driverAid.human()
            driverAid.update()
            goToHumanRPathFollower()
        }
        .whileState(statesRR.GO_TO_HUMAN_R2, { follower.atParametricEnd() }) {
            updateAll()
        }
        .transition(statesRR.GO_TO_HUMAN_R2, { follower.atParametricEnd() }, 0.5)
        .state(statesRR.GO_TO_SPECIMEN_R2)
        .onEnter(statesRR.GO_TO_SPECIMEN_R2) {
            goToSpecimenRCurve()
            driverAid.highSpecimen()
            driverAid.update()
        }
        .whileState(statesRR.GO_TO_SPECIMEN_R2, { follower.atParametricEnd() }) {
            updateAll()
        }
        .transition(statesRR.GO_TO_SPECIMEN_R2, { follower.atParametricEnd() }, 0.5)
        .state(statesRR.PLACE_SPECIMEN_R2)
        .onEnter(statesRR.PLACE_SPECIMEN_R2) {
            armSubsystem.setExtendTarget(900.0)
        }
        .whileState(statesRR.PLACE_SPECIMEN_R2, {
            armSubsystem.isExtendAtTarget(30.0)
        }) {
            updateAll()
        }
        .transition(
            statesRR.PLACE_SPECIMEN_R2,
            {
                (true)
            },
            0.0
        )
        .state(statesRR.GO_TO_HUMAN_R3)
        .onEnter(statesRR.GO_TO_HUMAN_R3) {
            driverAid.human()
            driverAid.update()
            goToHumanRPathFollower()
        }
        .whileState(statesRR.GO_TO_HUMAN_R3, { follower.atParametricEnd() }) {
            updateAll()
        }
        .transition(statesRR.GO_TO_HUMAN_R3, { follower.atParametricEnd() }, 0.5)
        .state(statesRR.GO_TO_SPECIMEN_R3)
        .onEnter(statesRR.GO_TO_SPECIMEN_R3) {
            goToSpecimenRCurve()
            driverAid.highSpecimen()
            driverAid.update()
        }
        .whileState(statesRR.GO_TO_SPECIMEN_R3, { follower.atParametricEnd() }) {
            updateAll()
        }
        .transition(statesRR.GO_TO_SPECIMEN_R3, { follower.atParametricEnd() }, 0.5)
        .state(statesRR.PLACE_SPECIMEN_R3)
        .onEnter(statesRR.PLACE_SPECIMEN_R3) {
            armSubsystem.setExtendTarget(900.0)
        }
        .whileState(statesRR.PLACE_SPECIMEN_R3, {
            armSubsystem.isExtendAtTarget(30.0)
        }) {
            updateAll()
        }
        .transition(
            statesRR.PLACE_SPECIMEN_R3,
            {
                (true)
            },
            0.0
        )
        .state(statesRR.GO_TO_END_R)
        .onEnter(statesRR.GO_TO_END_R) {
            driverAid.collapse()
            driverAid.update()
            goToEndRlPathFollower()
        }
        .whileState(statesRR.GO_TO_END_R, { follower.atParametricEnd() }) {
            updateAll()
        }
        .transition(statesRR.GO_TO_END_R, { follower.atParametricEnd() }, 0.5)
        .stopRunning(statesRR.STOP)
        .build()

    enum class statesRB {
        DRIVE_TO_SPECIMEN_B,
        PLACE_SPECIMEN_B,
        GO_TO_SAMPLE_B,
        GO_TO_SAMPLE_B2,
        GO_TO_SAMPLE_B3,
        GO_TO_HUMAN_B,
        GO_TO_SPECIMEN_B,
        PLACE_SPECIMEN_B1,
        GO_TO_HUMAN_B2,
        GO_TO_SPECIMEN_B2,
        PLACE_SPECIMEN_B2,
        GO_TO_HUMAN_B3,
        GO_TO_SPECIMEN_B3,
        PLACE_SPECIMEN_B3,
        GO_TO_END_B,
        STOP,
    }

    val smallSpecimenAutoB: StateMachine<statesRB> = StateMachine.Builder<statesRB>()
        .state(statesRB.DRIVE_TO_SPECIMEN_B)
        .onEnter(statesRB.DRIVE_TO_SPECIMEN_B) {
            driverAid.highSpecimen()
            placeSpecimenBPathFollower()
        }
        .whileState(statesRB.DRIVE_TO_SPECIMEN_B, { follower.atParametricEnd() }) {
            updateAll()
            telemetry.addData("x", follower.pose.x);
            telemetry.addData("y", follower.pose.y);
            telemetry.addData("heading", follower.pose.heading);
            telemetry.update();
        }
        .transition(statesRB.DRIVE_TO_SPECIMEN_B, { follower.atParametricEnd() }, 0.5)
        .state(statesRB.PLACE_SPECIMEN_B)
        .onEnter(statesRB.PLACE_SPECIMEN_B) {
            armSubsystem.setExtendTarget(900.0)
        }
        .whileState(statesRB.PLACE_SPECIMEN_B, {
            armSubsystem.isExtendAtTarget(30.0)
        }) {
            updateAll()
        }
        .transition(
            statesRB.PLACE_SPECIMEN_B,
            {
                (true)
            },
            0.0
        )
        .stopRunning(statesRB.STOP)
        .build()

    val fullSpecimenAutoB: StateMachine<statesRB> = StateMachine.Builder<statesRB>()
        .state(statesRB.DRIVE_TO_SPECIMEN_B)
        .onEnter(statesRB.DRIVE_TO_SPECIMEN_B) {
            driverAid.highSpecimen()
            placeSpecimenBPathFollower()
        }
        .whileState(statesRB.DRIVE_TO_SPECIMEN_B, { follower.atParametricEnd() }) {
            updateAll()
            telemetry.addData("x", follower.pose.x);
            telemetry.addData("y", follower.pose.y);
            telemetry.addData("heading", follower.pose.heading);
            telemetry.update();
        }
        .transition(statesRB.DRIVE_TO_SPECIMEN_B, { follower.atParametricEnd() }, 0.5)
        .state(statesRB.PLACE_SPECIMEN_B)
        .onEnter(statesRB.PLACE_SPECIMEN_B) {
            armSubsystem.setExtendTarget(900.0)
        }
        .whileState(statesRB.PLACE_SPECIMEN_B, {
            armSubsystem.isExtendAtTarget(30.0)
        }) {
            updateAll()
        }
        .transition(
            statesRB.PLACE_SPECIMEN_B,
            {
                (true)
            },
            0.0
        )
        .state(statesRB.GO_TO_SAMPLE_B)
        .onEnter(statesRB.GO_TO_SAMPLE_B) {
            goToSampleBPathFollower(0.0)
        }
        .whileState(statesRB.GO_TO_SAMPLE_B, { follower.atParametricEnd() }) {
            updateAll()
        }
        .transition(statesRB.GO_TO_SAMPLE_B, { follower.atParametricEnd() }, 0.5)
        .state(statesRB.GO_TO_SAMPLE_B2)
        .onEnter(statesRB.GO_TO_SAMPLE_B2) {
            goToSampleBPathFollower(0.0)
        }
        .whileState(statesRB.GO_TO_SAMPLE_B2, { follower.atParametricEnd() }) {
            updateAll()
        }
        .transition(statesRB.GO_TO_SAMPLE_B2, { follower.atParametricEnd() }, 0.5)
        .state(statesRB.GO_TO_SAMPLE_B3)
        .onEnter(statesRB.GO_TO_SAMPLE_B3) {
            goToSampleBPathFollower(0.0)
        }
        .whileState(statesRB.GO_TO_SAMPLE_B3, { follower.atParametricEnd() }) {
            updateAll()
        }
        .transition(statesRB.GO_TO_SAMPLE_B3, { follower.atParametricEnd() }, 0.5)
        .state(statesRB.GO_TO_HUMAN_B)
        .onEnter(statesRB.GO_TO_HUMAN_B) {
            driverAid.human()
            driverAid.update()
            goToHumanBPathFollower()
        }
        .whileState(statesRB.GO_TO_HUMAN_B, { follower.atParametricEnd() }) {
            updateAll()
        }
        .transition(statesRB.GO_TO_HUMAN_B, { follower.atParametricEnd() }, 0.5)
        .state(statesRB.GO_TO_SPECIMEN_B)
        .onEnter(statesRB.GO_TO_SPECIMEN_B) {
            goToSpecimenBCurve()
            driverAid.highSpecimen()
            driverAid.update()
        }
        .whileState(statesRB.GO_TO_SPECIMEN_B, { follower.atParametricEnd() }) {
            updateAll()
        }
        .transition(statesRB.GO_TO_SPECIMEN_B, { follower.atParametricEnd() }, 0.5)
        .state(statesRB.PLACE_SPECIMEN_B1)
        .onEnter(statesRB.PLACE_SPECIMEN_B1) {
            armSubsystem.setExtendTarget(900.0)
        }
        .whileState(statesRB.PLACE_SPECIMEN_B1, {
            armSubsystem.isExtendAtTarget(30.0)
        }) {
            updateAll()
        }
        .transition(
            statesRB.PLACE_SPECIMEN_B1,
            {
                (true)
            },
            0.0
        )
        .state(statesRB.GO_TO_HUMAN_B2)
        .onEnter(statesRB.GO_TO_HUMAN_B2) {
            driverAid.human()
            driverAid.update()
            goToHumanBPathFollower()
        }
        .whileState(statesRB.GO_TO_HUMAN_B2, { follower.atParametricEnd() }) {
            updateAll()
        }
        .transition(statesRB.GO_TO_HUMAN_B2, { follower.atParametricEnd() }, 0.5)
        .state(statesRB.GO_TO_SPECIMEN_B2)
        .onEnter(statesRB.GO_TO_SPECIMEN_B2) {
            goToSpecimenBCurve()
            driverAid.highSpecimen()
            driverAid.update()
        }
        .whileState(statesRB.GO_TO_SPECIMEN_B2, { follower.atParametricEnd() }) {
            updateAll()
        }
        .transition(statesRB.GO_TO_SPECIMEN_B2, { follower.atParametricEnd() }, 0.5)
        .state(statesRB.PLACE_SPECIMEN_B2)
        .onEnter(statesRB.PLACE_SPECIMEN_B2) {
            armSubsystem.setExtendTarget(900.0)
        }
        .whileState(statesRB.PLACE_SPECIMEN_B2, {
            armSubsystem.isExtendAtTarget(30.0)
        }) {
            updateAll()
        }
        .transition(
            statesRB.PLACE_SPECIMEN_B2,
            {
                (true)
            },
            0.0
        )
        .state(statesRB.GO_TO_HUMAN_B3)
        .onEnter(statesRB.GO_TO_HUMAN_B3) {
            driverAid.human()
            driverAid.update()
            goToHumanBPathFollower()
        }
        .whileState(statesRB.GO_TO_HUMAN_B3, { follower.atParametricEnd() }) {
            updateAll()
        }
        .transition(statesRB.GO_TO_HUMAN_B3, { follower.atParametricEnd() }, 0.5)
        .state(statesRB.GO_TO_SPECIMEN_B3)
        .onEnter(statesRB.GO_TO_SPECIMEN_B3) {
            goToSpecimenBCurve()
            driverAid.highSpecimen()
            driverAid.update()
        }
        .whileState(statesRB.GO_TO_SPECIMEN_B3, { follower.atParametricEnd() }) {
            updateAll()
        }
        .transition(statesRB.GO_TO_SPECIMEN_B3, { follower.atParametricEnd() }, 0.5)
        .state(statesRB.PLACE_SPECIMEN_B3)
        .onEnter(statesRB.PLACE_SPECIMEN_B3) {
            armSubsystem.setExtendTarget(900.0)
        }
        .whileState(statesRB.PLACE_SPECIMEN_B3, {
            armSubsystem.isExtendAtTarget(30.0)
        }) {
            updateAll()
        }
        .transition(
            statesRB.PLACE_SPECIMEN_B3,
            {
                (true)
            },
            0.0
        )
        .state(statesRB.GO_TO_END_B)
        .onEnter(statesRB.GO_TO_END_B) {
            driverAid.collapse()
            driverAid.update()
            goToEndBlPathFollower()
        }
        .whileState(statesRB.GO_TO_END_B, { follower.atParametricEnd() }) {
            updateAll()
        }
        .transition(statesRB.GO_TO_END_B, { follower.atParametricEnd() }, 0.5)
        .stopRunning(statesRB.STOP)
        .build()
}