package org.firstinspires.ftc.teamcode.customHardware

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.customHardware.Actions.pathBuilder
import org.firstinspires.ftc.teamcode.customHardware.Points.blueBasket
import org.firstinspires.ftc.teamcode.customHardware.Points.blueBasketAngle
import org.firstinspires.ftc.teamcode.customHardware.Points.blueEndAngle
import org.firstinspires.ftc.teamcode.customHardware.Points.blueEndLeft
import org.firstinspires.ftc.teamcode.customHardware.Points.blueEndRight
import org.firstinspires.ftc.teamcode.customHardware.Points.blueHuman
import org.firstinspires.ftc.teamcode.customHardware.Points.blueNeutralSample
import org.firstinspires.ftc.teamcode.customHardware.Points.blueSample
import org.firstinspires.ftc.teamcode.customHardware.Points.blueSampleAngle
import org.firstinspires.ftc.teamcode.customHardware.Points.blueSpecimen
import org.firstinspires.ftc.teamcode.customHardware.Points.blueSpecimenAngle
import org.firstinspires.ftc.teamcode.customHardware.Points.redBasket
import org.firstinspires.ftc.teamcode.customHardware.Points.redBasketAngle
import org.firstinspires.ftc.teamcode.customHardware.Points.redEndAngle
import org.firstinspires.ftc.teamcode.customHardware.Points.redEndLeft
import org.firstinspires.ftc.teamcode.customHardware.Points.redEndRight
import org.firstinspires.ftc.teamcode.customHardware.Points.redHuman
import org.firstinspires.ftc.teamcode.customHardware.Points.redNeutralSample
import org.firstinspires.ftc.teamcode.customHardware.Points.redSample
import org.firstinspires.ftc.teamcode.customHardware.Points.redSampleAngle
import org.firstinspires.ftc.teamcode.customHardware.Points.redSpecimen
import org.firstinspires.ftc.teamcode.customHardware.Points.redSpecimenAngle
import org.firstinspires.ftc.teamcode.customHardware.Traj.goToBasketBCurve
import org.firstinspires.ftc.teamcode.customHardware.Traj.goToBasketRCurve
import org.firstinspires.ftc.teamcode.customHardware.Traj.goToHumanBCurve
import org.firstinspires.ftc.teamcode.customHardware.Traj.goToHumanRCurve
import org.firstinspires.ftc.teamcode.customHardware.Traj.goToSpecimenBCurve
import org.firstinspires.ftc.teamcode.customHardware.Traj.goToSpecimenRCurve
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.followers.pedroPathing.follower.Follower
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.BezierCurve
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.PathBuilder
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.Point
import org.firstinspires.ftc.teamcode.subsystems.DriverAid
import org.firstinspires.ftc.teamcode.utilClass.storage.PoseStorage
import org.gentrifiedApps.statemachineftc.StateMachine

class AutoHardware(
    opmode: LinearOpMode,
    startLocation: StartLocation,
    ahwMap: HardwareMap = opmode.hardwareMap,
) : HardwareConfig(opmode, true, startLocation, ahwMap) {
    lateinit var follower: Follower

    init {
        val robot = HardwareConfig(opmode, true, startLocation)
        follower = robot.localizerSubsystem.follower
        telemetry.addData("Status", "Initialized")
        telemetry.addData("Alliance", startLocation.alliance)
        telemetry.update()
        opmode.waitForStart()
        timer.reset()
    }
}

object Points {
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
}

object Traj {
    val goToSpecimenBCurve =
        BezierCurve(
            PoseStorage.currentPoint,
            Point(24.0, 72.0),
            blueSpecimen
        )
    val goToSpecimenRCurve =
        BezierCurve(
            PoseStorage.currentPoint,
            Point(126.0, 72.0),
            redSpecimen
        )
    val goToBasketBCurve =
        BezierCurve(
            PoseStorage.currentPoint,
            Point(36.0, 120.0),
            blueBasket
        )
    val goToBasketRCurve =
        BezierCurve(
            PoseStorage.currentPoint,
            Point(108.0, 24.0),
            redBasket
        )
    val goToNeutralSampleBCurve =
        BezierCurve(
            PoseStorage.currentPoint,
            Point(30.0, 120.0),
            blueNeutralSample
        )
    val goToNeutralSampleRCurve =
        BezierCurve(
            PoseStorage.currentPoint,
            Point(114.0, 24.0),
            redNeutralSample
        )
    val goToSampleBCurve =
        BezierCurve(
            PoseStorage.currentPoint,
            Point(64.0, 22.0),
            blueSample
        )
    val goToSampleRCurve =
        BezierCurve(
            PoseStorage.currentPoint,
            Point(84.0, 132.0),
            redSample
        )
    val goToEndBlCurve =
        BezierCurve(
            PoseStorage.currentPoint,
            Point(26.0, 120.0),
            Point(64.0, 114.0),
            blueEndLeft
        )
    val goToEndBrCurve =
        BezierCurve(
            PoseStorage.currentPoint,
            Point(26.0, 120.0),
            Point(64.0, 114.0),
            blueEndRight
        )
    val goToEndRlCurve =
        BezierCurve(
            PoseStorage.currentPoint,
            Point(118.0, 24.0),
            Point(80.0, 30.0),
            redEndLeft
        )
    val goToEndRrCurve =
        BezierCurve(
            PoseStorage.currentPoint,
            Point(118.0, 24.0),
            Point(80.0, 30.0),
            redEndRight
        )
    val goToHumanBCurve =
        BezierCurve(
            PoseStorage.currentPoint,
            Point(34.0, 34.0),
            blueHuman
        )
    val goToHumanRCurve =
        BezierCurve(
            PoseStorage.currentPoint,
            Point(108.0, 108.0),
            redHuman
        )
}

object Actions {
    val pathBuilder = PathBuilder()
    val placeSpecimenBPath = pathBuilder.addPath(
        goToSpecimenBCurve,
    ).setLinearHeadingInterpolation(PoseStorage.currentHeading, blueSpecimenAngle)
        .build()
    val placeSpecimenRPath = pathBuilder.addPath(
        goToSpecimenRCurve,
    ).setLinearHeadingInterpolation(PoseStorage.currentHeading, redSpecimenAngle)
        .build()
    val placeBasketBPath = pathBuilder.addPath(
        goToBasketBCurve,
    ).setLinearHeadingInterpolation(PoseStorage.currentHeading, blueBasketAngle)
        .build()
    val placeBasketRPath = pathBuilder.addPath(
        goToBasketRCurve,
    ).setLinearHeadingInterpolation(PoseStorage.currentHeading, redBasketAngle)
        .build()
    val getSpecimenRPath = pathBuilder.addPath(
        goToHumanRCurve,
    ).setLinearHeadingInterpolation(PoseStorage.currentHeading, redSpecimenAngle)
        .build()
    val getSpecimenBPath = pathBuilder.addPath(
        goToHumanBCurve,
    ).setLinearHeadingInterpolation(PoseStorage.currentHeading, blueSpecimenAngle)
        .build()

    val getSampleRPath = pathBuilder.addPath(
        Traj.goToSampleRCurve
    ).setLinearHeadingInterpolation(PoseStorage.currentHeading, redSampleAngle)
        .build()
    val getSampleBPath = pathBuilder.addPath(
        Traj.goToSampleBCurve
    ).setLinearHeadingInterpolation(PoseStorage.currentHeading, blueSampleAngle)
        .build()

    val goToEndBlPath = pathBuilder.addPath(
        Traj.goToEndBlCurve
    ).setLinearHeadingInterpolation(PoseStorage.currentHeading, blueEndAngle)
        .build()
    val goToEndBrPath = pathBuilder.addPath(
        Traj.goToEndBrCurve
    ).setLinearHeadingInterpolation(PoseStorage.currentHeading, blueEndAngle)
        .build()
    val goToEndRlPath = pathBuilder.addPath(
        Traj.goToEndRlCurve
    ).setLinearHeadingInterpolation(PoseStorage.currentHeading, redEndAngle)
        .build()
    val goToEndRrPath = pathBuilder.addPath(
        Traj.goToEndRrCurve
    ).setLinearHeadingInterpolation(PoseStorage.currentHeading, redEndAngle)
        .build()
}

class Followers {
    private fun refreshPose(follower: Follower) {
        PoseStorage.currentPose = follower.pose
    }

    fun placeSpecimenBPathFollower(follower: Follower) {
        follower.followPath(Actions.placeSpecimenBPath)
        refreshPose(follower)
    }

    fun placeSpecimenRPathFollower(follower: Follower) {
        follower.followPath(Actions.placeSpecimenRPath)
        refreshPose(follower)
    }

    fun goToBasketBPathFollower(follower: Follower) {
        follower.followPath(Actions.placeBasketBPath)
        refreshPose(follower)
    }

    fun goToBasketRPathFollower(follower: Follower) {
        follower.followPath(Actions.placeBasketRPath)
        refreshPose(follower)
    }

    fun goToSampleBPathFollower(follower: Follower) {
        follower.followPath(Actions.getSampleBPath)
        refreshPose(follower)
    }

    fun goToSampleRPathFollower(follower: Follower) {
        follower.followPath(Actions.getSampleRPath)
        refreshPose(follower)
    }

    fun goToEndBlPathFollower(follower: Follower) {
        follower.followPath(Actions.goToEndBlPath)
        refreshPose(follower)
    }

    fun goToEndBrPathFollower(follower: Follower) {
        follower.followPath(Actions.goToEndBrPath)
        refreshPose(follower)
    }

    fun goToEndRlPathFollower(follower: Follower) {
        follower.followPath(Actions.goToEndRlPath)
        refreshPose(follower)
    }

    fun goToEndRrPathFollower(follower: Follower) {
        follower.followPath(Actions.goToEndRrPath)
        refreshPose(follower)
    }

    fun goToHumanBPathFollower(follower: Follower) {
        follower.followPath(Actions.getSpecimenBPath)
        refreshPose(follower)
    }

    fun goToHumanRPathFollower(follower: Follower) {
        follower.followPath(Actions.getSpecimenRPath)
        refreshPose(follower)
    }

    fun goToNeutralSampleBPathFollower(follower: Follower, angle: Double) {
        val path = pathBuilder.addPath(
            Traj.goToNeutralSampleBCurve
        ).setLinearHeadingInterpolation(PoseStorage.currentPose.heading.toDouble(), angle)
            .build()
        follower.followPath(path)
        refreshPose(follower)
    }

    fun goToNeutralSampleRPathFollower(follower: Follower, angle: Double) {
        val path = pathBuilder.addPath(
            Traj.goToNeutralSampleRCurve
        ).setLinearHeadingInterpolation(PoseStorage.currentPose.heading.toDouble(), angle)
            .build()
        follower.followPath(path)
        refreshPose(follower)
    }
}


class SMs(follower: Follower) {
    enum class states {
        driveToSpecimenB,
        placeSpecimenB,
        stop,
    }

    init {
        val sm1 = StateMachine.Builder<states>()
            .state(states.driveToSpecimenB)
            .onEnter(states.driveToSpecimenB) {
                Followers().placeSpecimenBPathFollower(follower)
            }.whileState(states.driveToSpecimenB, { follower.atParametricEnd() }) {
                follower.update()
            }.transition(states.placeSpecimenB, { follower.atParametricEnd() }, 0.0)
            .state(states.placeSpecimenB)
            .onEnter(states.placeSpecimenB) {
                DriverAid.collapseSM.start()
            }.whileState(states.placeSpecimenB, { !DriverAid.collapseSM.isRunning }) {
                DriverAid.collapseSM.update()
            }.transition(states.placeSpecimenB, { !DriverAid.collapseSM.isRunning }, 0.0)
            .stopRunning(states.stop)
            .build()
    }

}
