package org.firstinspires.ftc.teamcode.customHardware

import com.acmerobotics.roadrunner.Pose2d
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
import org.firstinspires.ftc.teamcode.customHardware.Points.redSpecimen
import org.firstinspires.ftc.teamcode.customHardware.Points.redSpecimenAngle
import org.firstinspires.ftc.teamcode.customHardware.Traj.goToBasketB
import org.firstinspires.ftc.teamcode.customHardware.Traj.goToBasketR
import org.firstinspires.ftc.teamcode.customHardware.Traj.goToHumanB
import org.firstinspires.ftc.teamcode.customHardware.Traj.goToHumanR
import org.firstinspires.ftc.teamcode.customHardware.Traj.goToNeutralSampleB
import org.firstinspires.ftc.teamcode.customHardware.Traj.goToNeutralSampleR
import org.firstinspires.ftc.teamcode.customHardware.Traj.goToSpecimenB
import org.firstinspires.ftc.teamcode.customHardware.Traj.goToSpecimenR
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.followers.pedroPathing.follower.Follower
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.BezierCurve
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.PathBuilder
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.Point
import org.firstinspires.ftc.teamcode.utilClass.storage.PoseStorage

class AutoHardware(
    opmode: LinearOpMode,
    startLocation: StartLocation,
    ahwMap: HardwareMap = opmode.hardwareMap,
) : HardwareConfig(opmode, true, startLocation, ahwMap) {


    init {
        val robot = HardwareConfig(opmode, true, startLocation)
        val follower = robot.localizerSubsystem.follower
        follower.setStartingPose(PoseStorage.currentPose)
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
    val goToSpecimenB =
        BezierCurve(
            PoseStorage.currentPoint,
            Point(24.0, 72.0),
            blueSpecimen
        )
    val goToSpecimenR =
        BezierCurve(
            PoseStorage.currentPoint,
            Point(126.0, 72.0),
            redSpecimen
        )
    val goToBasketB =
        BezierCurve(
            PoseStorage.currentPoint,
            Point(36.0, 120.0),
            blueBasket
        )
    val goToBasketR =
        BezierCurve(
            PoseStorage.currentPoint,
            Point(108.0, 24.0),
            redBasket
        )
    val goToNeutralSampleB =
        BezierCurve(
            PoseStorage.currentPoint,
            Point(30.0, 120.0),
            blueNeutralSample
        )
    val goToNeutralSampleR =
        BezierCurve(
            PoseStorage.currentPoint,
            Point(114.0, 24.0),
            redNeutralSample
        )
    val goToSampleB =
        BezierCurve(
            PoseStorage.currentPoint,
            Point(64.0, 22.0),
            blueSample
        )
    val goToSampleR =
        BezierCurve(
            PoseStorage.currentPoint,
            Point(84.0, 132.0),
            redSample
        )
    val goToEndBl =
        BezierCurve(
            PoseStorage.currentPoint,
            Point(26.0, 120.0),
            Point(64.0, 114.0),
            blueEndLeft
        )
    val goToEndBr =
        BezierCurve(
            PoseStorage.currentPoint,
            Point(26.0, 120.0),
            Point(64.0, 114.0),
            blueEndRight
        )
    val goToEndRl =
        BezierCurve(
            PoseStorage.currentPoint,
            Point(118.0, 24.0),
            Point(80.0, 30.0),
            redEndLeft
        )
    val goToEndRr =
        BezierCurve(
            PoseStorage.currentPoint,
            Point(118.0, 24.0),
            Point(80.0, 30.0),
            redEndRight
        )
    val goToHumanB =
        BezierCurve(
            PoseStorage.currentPoint,
            Point(34.0, 34.0),
            blueHuman
        )
    val goToHumanR =
        BezierCurve(
            PoseStorage.currentPoint,
            Point(108.0, 108.0),
            redHuman
        )

}

object Actions {
    val pathBuilder = PathBuilder()
    val placeSpecimenB = pathBuilder.addPath(
        goToSpecimenB,
    ).setLinearHeadingInterpolation(PoseStorage.currentHeading, blueSpecimenAngle)
        .addParametricCallback(0.5) {
//        scoringSubsystem.placeSpecimen()
        }.build()
    val placeSpecimenR = pathBuilder.addPath(
        goToSpecimenR,
    ).setLinearHeadingInterpolation(PoseStorage.currentHeading, redSpecimenAngle)
        .addParametricCallback(0.5) {
//        scoringSubsystem.placeSpecimen()
        }.build()
    val placeBasketB = pathBuilder.addPath(
        goToBasketB,
    ).setLinearHeadingInterpolation(PoseStorage.currentHeading, blueBasketAngle)
        .addParametricCallback(0.5) {
//        scoringSubsystem.placeBasket()
        }.build()
    val placeBasketR = pathBuilder.addPath(
        goToBasketR,
    ).setLinearHeadingInterpolation(PoseStorage.currentHeading, redBasketAngle)
        .addParametricCallback(0.5) {
//        scoringSubsystem.placeBasket()
        }.build()
    val getSpecimenR = pathBuilder.addPath(
        goToHumanR,
    ).setLinearHeadingInterpolation(PoseStorage.currentHeading, redSpecimenAngle)
        .addParametricCallback(0.5) {
//        scoringSubsystem.getSpecimen()
        }.build()
    val getSpecimenB = pathBuilder.addPath(
        goToHumanB,
    ).setLinearHeadingInterpolation(PoseStorage.currentHeading, blueSpecimenAngle)
        .addParametricCallback(0.5) {
//        scoringSubsystem.getSpecimen()
        }.build()
    val getNeutralSampleB = pathBuilder.addPath(
        goToNeutralSampleB
    ).setLinearHeadingInterpolation(PoseStorage.currentHeading, 30.0)//Todo
        .addParametricCallback(0.5) {

        }.build()
    val getNeutralSampleR = pathBuilder.addPath(
        goToNeutralSampleR
    ).setLinearHeadingInterpolation(PoseStorage.currentHeading, 30.0)//TODO
        .addParametricCallback(0.5) {

        }.build()

    val goToEndBl = pathBuilder.addPath(
        Traj.goToEndBl
    ).setLinearHeadingInterpolation(PoseStorage.currentHeading, blueEndAngle)
        .addParametricCallback(0.5) {

        }.build()
    val goToEndBr = pathBuilder.addPath(
        Traj.goToEndBr
    ).setLinearHeadingInterpolation(PoseStorage.currentHeading, blueEndAngle)
        .addParametricCallback(0.5) {

        }.build()
    val goToEndRl = pathBuilder.addPath(
        Traj.goToEndRl
    ).setLinearHeadingInterpolation(PoseStorage.currentHeading, redEndAngle)
        .addParametricCallback(0.5) {

        }.build()
    val goToEndRr = pathBuilder.addPath(
        Traj.goToEndRr
    ).setLinearHeadingInterpolation(PoseStorage.currentHeading, redEndAngle)
        .addParametricCallback(0.5) {

        }.build()
}

class TrajectoryChains(var follower: Follower) {
    private fun refreshPose() {
        PoseStorage.currentPose = follower.pose
    }

    fun placeSpecimenB() {
        follower.followPath(Actions.placeSpecimenB)
        refreshPose()
    }

    fun placeSpecimenR() {
        follower.followPath(Actions.placeSpecimenR)
        refreshPose()
    }

    fun goToBasketB() {
        follower.followPath(Actions.placeBasketB)
        refreshPose()
    }

    fun goToBasketR() {
        follower.followPath(Actions.placeBasketR)
        refreshPose()
    }

    fun goToNeutralSampleB() {
        follower.followPath(Actions.getNeutralSampleB)
        refreshPose()
    }

    fun goToNeutralSampleR() {
        follower.followPath(Actions.getNeutralSampleR)
        refreshPose()
    }

    fun goToSampleB() {
        follower.followPath(Actions.getNeutralSampleB)
        refreshPose()
    }

    fun goToSampleR() {
        follower.followPath(Actions.getNeutralSampleR)
        refreshPose()
    }

    fun goToEndBl() {
        follower.followPath(Actions.goToEndBl)
        refreshPose()
    }

    fun goToEndBr() {
        follower.followPath(Actions.goToEndBr)
        refreshPose()
    }

    fun goToEndRl() {
        follower.followPath(Actions.goToEndRl)
        refreshPose()
    }

    fun goToEndRr() {
        follower.followPath(Actions.goToEndRr)
        refreshPose()
    }

    fun goToHumanB() {
        follower.followPath(Actions.getSpecimenB)
        refreshPose()
    }

    fun goToHumanR() {
        follower.followPath(Actions.getSpecimenR)
        refreshPose()
    }

    fun getSampleB(angle: Double) {
        val path = pathBuilder.addPath(
            Traj.goToSampleB
        ).setLinearHeadingInterpolation(PoseStorage.currentPose.heading.toDouble(), angle)
            .addParametricCallback(0.5) {

            }.build()
        follower.followPath(path)
        refreshPose()
    }

    fun getSampleR(angle: Double) {
        val path = pathBuilder.addPath(
            Traj.goToSampleR
        ).setLinearHeadingInterpolation(PoseStorage.currentPose.heading.toDouble(), angle)
            .addParametricCallback(0.5) {

            }.build()
        follower.followPath(path)
        refreshPose()
    }
}

fun Pose2d.toPoint2() = Point(this.position.x, this.position.y)