package org.firstinspires.ftc.teamcode.customHardware

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.followers.pedroPathing.follower.Follower
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.BezierCurve
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.PathBuilder
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.PathChain
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.Point
import org.firstinspires.ftc.teamcode.utilClass.storage.PoseStorage
import org.gentrifiedApps.statemachineftc.StateMachine

class AutoHardware(
    opmode: LinearOpMode,
    startLocation: StartLocation,
    ahwMap: HardwareMap = opmode.hardwareMap,
) : HardwareConfig(opmode, true, startLocation, ahwMap) {
    val follower: Follower = this.localizerSubsystem.follower

    init {
        telemetry.addData("Status", "Initialized")
        telemetry.addData("Alliance", startLocation.alliance)
        telemetry.update()
        opmode.waitForStart()
        timer.reset()
    }

    fun autoSetup() {
        this.scoringSubsystem.setup()
    }


    val blueX = 6.0
    val blueStartLeft = Point(blueX, 108.0)
    val blueStartRight = Point(blueX, 60.0)
    val blueStartAngle = Math.toRadians(0.0)
    val redX = 136.0
    val redStartLeft = Point(redX, 88.0)
    val redStartRight = Point(redX, 36.0)
    val redStartAngle = Math.toRadians(180.0)

    val blueEndLeft = Point(62.0, 96.0)
    val blueEndRight = Point(62.0, 108.0)
    val blueEndAngle = Math.toRadians(90.0)
    val redEndLeft = Point(62.0, 48.0)
    val redEndRight = Point(82.0, 48.0)
    val redEndAngle = Math.toRadians(-90.0)
    val blueHuman = Point(12.0, 24.0)
    val blueHumanAngle = Math.toRadians(180.0)
    val redHuman = Point(130.0, 120.0)
    val redHumanAngle = Math.toRadians(0.0)
    val blueBasket = Point(22.0, 122.0)
    val blueBasketAngle = Math.toRadians(135.0)
    val redBasket = Point(122.0, 22.0)
    val redBasketAngle = Math.toRadians(-45.0)
    val blueSpecimen = Point(36.0, 72.0)
    val blueSpecimenAngle = Math.toRadians(0.0)
    val redSpecimen = Point(108.0, 72.0)
    val redSpecimenAngle = Math.toRadians(180.0)
    val blueSample = Point(60.0, 12.0)
    val blueSampleAngle = Math.toRadians(90.0)
    val redSample = Point(84.0, 132.0)
    val redSampleAngle = Math.toRadians(90.0)
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

    val pathBuilder = PathBuilder()

    fun placeSpecimenBPath(): PathChain {
        return pathBuilder.addPath(
            goToSpecimenBCurve()
        ).setLinearHeadingInterpolation(PoseStorage.currentHeading, blueSpecimenAngle)
            .build()
    }

    fun placeSpecimenRPath(): PathChain {
        return pathBuilder.addPath(
            goToSpecimenRCurve()
        ).setLinearHeadingInterpolation(PoseStorage.currentHeading, redSpecimenAngle)
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

    private fun refreshPose(follower: Follower) {
        PoseStorage.currentPose = follower.pose
    }

    fun placeSpecimenBPathFollower(follower: Follower) {
        follower.followPath(placeSpecimenBPath())
        refreshPose(follower)
    }

    fun placeSpecimenRPathFollower(follower: Follower) {
        follower.followPath(placeSpecimenRPath())
        refreshPose(follower)
    }

    fun goToBasketBPathFollower(follower: Follower) {
        follower.followPath(placeBasketBPath())
        refreshPose(follower)
    }

    fun goToBasketRPathFollower(follower: Follower) {
        follower.followPath(placeBasketRPath())
        refreshPose(follower)
    }

    fun goToSampleBPathFollower(follower: Follower, offsetY: Double) {
        follower.followPath(getSampleBPath(offsetY))
        refreshPose(follower)
    }

    fun goToSampleRPathFollower(follower: Follower, offsetY: Double) {
        follower.followPath(getSampleRPath(offsetY))
        refreshPose(follower)
    }

    fun goToEndBlPathFollower(follower: Follower) {
        follower.followPath(goToEndBlPath())
        refreshPose(follower)
    }

    fun goToEndBrPathFollower(follower: Follower) {
        follower.followPath(goToEndBrPath())
        refreshPose(follower)
    }

    fun goToEndRlPathFollower(follower: Follower) {
        follower.followPath(goToEndRlPath())
        refreshPose(follower)
    }

    fun goToEndRrPathFollower(follower: Follower) {
        follower.followPath(goToEndRrPath())
        refreshPose(follower)
    }

    fun goToHumanBPathFollower(follower: Follower) {
        follower.followPath(getSpecimenBPath())
        refreshPose(follower)
    }

    fun goToHumanRPathFollower(follower: Follower) {
        follower.followPath(getSpecimenRPath())
        refreshPose(follower)
    }

    fun goToNeutralSampleBPathFollower(follower: Follower, angle: Double) {
        follower.followPath(goToNeutralSampleB(angle))
        refreshPose(follower)
    }

    fun goToNeutralSampleRPathFollower(follower: Follower, angle: Double) {
        follower.followPath(goToNeutralSampleR(angle))
        refreshPose(follower)
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

    var rr: StateMachine<statesRR> = StateMachine.Builder<statesRR>()
        .state(statesRR.driveToSpecimenR)
        .onEnter(statesRR.driveToSpecimenR) {
            placeSpecimenBPathFollower(follower)
        }
        .whileState(statesRR.driveToSpecimenR, { follower.atParametricEnd() }) {
            follower.update()
        }
        .transition(statesRR.driveToSpecimenR, { follower.atParametricEnd() }, 0.0)
//            .state(statesRR.placeSpecimenR)
//            .onEnter(statesRR.placeSpecimenR) {
//                DriverAid.collapseSM.start()
//            }
//            .whileState(statesRR.placeSpecimenR, { !DriverAid.collapseSM.isRunning }) {
//                DriverAid.collapseSM.update()
//            }
//            .transition(statesRR.placeSpecimenR, { !DriverAid.collapseSM.isRunning }, 0.0)
        .state(statesRR.goToSampleR)
        .onEnter(statesRR.goToSampleR) {
            goToSampleRPathFollower(follower, 0.0)
        }
        .whileState(statesRR.goToSampleR, { follower.atParametricEnd() }) {
            follower.update()
        }
        .transition(statesRR.goToSampleR, { follower.atParametricEnd() }, 0.0)
        .state(statesRR.goToSampleR2)
        .onEnter(statesRR.goToSampleR2) {
            goToSampleRPathFollower(follower, 6.0)
        }
        .whileState(statesRR.goToSampleR2, { follower.atParametricEnd() }) {
            follower.update()
        }
        .transition(statesRR.goToSampleR2, { follower.atParametricEnd() }, 0.0)
        .state(statesRR.goToSampleR3)
        .onEnter(statesRR.goToSampleR3) {
            goToSampleRPathFollower(follower, -6.0)
        }
        .whileState(statesRR.goToSampleR3, { follower.atParametricEnd() }) {
            follower.update()
        }
        .transition(statesRR.goToSampleR3, { follower.atParametricEnd() }, 0.0)
        .state(statesRR.goToHumanR)
        .onEnter(statesRR.goToHumanR) {
            goToHumanRPathFollower(follower)
        }
        .whileState(statesRR.goToHumanR, { follower.atParametricEnd() }) {
            follower.update()
        }
        .transition(statesRR.goToHumanR, { follower.atParametricEnd() }, 0.0)
        .state(statesRR.goToSpecimenR)
        .onEnter(statesRR.goToSpecimenR) {
            placeSpecimenRPathFollower(follower)
        }
        .whileState(statesRR.goToSpecimenR, { follower.atParametricEnd() }) {
            follower.update()
        }
        .transition(statesRR.goToSpecimenR, { follower.atParametricEnd() }, 0.0)
//            .state(statesRR.placeSpecimenR1)
//            .onEnter(statesRR.placeSpecimenR1) {
//                DriverAid.collapseSM.start()
//            }
//            .whileState(statesRR.placeSpecimenR1, { !DriverAid.collapseSM.isRunning }) {
//                DriverAid.collapseSM.update()
//            }
//            .transition(statesRR.placeSpecimenR1, { !DriverAid.collapseSM.isRunning }, 0.0)
        .state(statesRR.goToHumanR2)
        .onEnter(statesRR.goToHumanR2) {
            goToHumanRPathFollower(follower)
        }
        .whileState(statesRR.goToHumanR2, { follower.atParametricEnd() }) {
            follower.update()
        }
        .transition(statesRR.goToHumanR2, { follower.atParametricEnd() }, 0.0)
        .state(statesRR.goToSpecimenR2)
        .onEnter(statesRR.goToSpecimenR2) {
            placeSpecimenRPathFollower(follower)
        }
        .whileState(statesRR.goToSpecimenR2, { follower.atParametricEnd() }) {
            follower.update()
        }
        .transition(statesRR.goToSpecimenR2, { follower.atParametricEnd() }, 0.0)
//            .state(statesRR.placeSpecimenR2)
//            .onEnter(statesRR.placeSpecimenR2) {
//                DriverAid.collapseSM.start()
//            }
//            .whileState(statesRR.placeSpecimenR2, { !DriverAid.collapseSM.isRunning }) {
//                DriverAid.collapseSM.update()
//            }
//            .transition(statesRR.placeSpecimen2R, { !DriverAid.collapseSM.isRunning }, 0.0)
        .state(statesRR.goToHumanR3)
        .onEnter(statesRR.goToHumanR3) {
            goToHumanRPathFollower(follower)
        }
        .whileState(statesRR.goToHumanR3, { follower.atParametricEnd() }) {
            follower.update()
        }
        .transition(statesRR.goToHumanR3, { follower.atParametricEnd() }, 0.0)
        .state(statesRR.goToSpecimenR3)
        .onEnter(statesRR.goToSpecimenR3) {
            placeSpecimenRPathFollower(follower)
        }
        .whileState(statesRR.goToSpecimenR3, { follower.atParametricEnd() }) {
            follower.update()
        }
        .transition(statesRR.goToSpecimenR3, { follower.atParametricEnd() }, 0.0)
//            .state(statesRR.placeSpecimenR3)
//            .onEnter(statesRR.placeSpecimenR3) {
//                DriverAid.collapseSM.start()
//            }
//            .whileState(statesRR.placeSpecimenR3, { !DriverAid.collapseSM.isRunning }) {
//                DriverAid.collapseSM.update()
//            }
//            .transition(statesRR.placeSpecimenR3, { !DriverAid.collapseSM.isRunning }, 0.0)
        .state(statesRR.goToEndR)
        .onEnter(statesRR.goToEndR) {
            goToEndRrPathFollower(follower)
        }
        .whileState(statesRR.goToEndR, { follower.atParametricEnd() }) {
            follower.update()
        }
        .onExit(statesRR.goToEndR) {
        }
        .transition(statesRR.goToEndR, { follower.atParametricEnd() }, 0.0)
        .stopRunning(statesRR.stop)
        .build()

}