package org.firstinspires.ftc.teamcode.opModes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.customHardware.AutoHardware
import org.firstinspires.ftc.teamcode.followers.pedroPathing.follower.Follower
import org.firstinspires.ftc.teamcode.followers.pedroPathing.localization.Pose
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.BezierCurve
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.Point
import org.firstinspires.ftc.teamcode.followers.pedroPathing.tuning.Circle


@Autonomous
class ppTestAuto : LinearOpMode() {
    override fun runOpMode() {
        val follower = Follower(hardwareMap)

        val circle = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(0.0, 0.0, Point.CARTESIAN),
                    Point(Circle.RADIUS, 0.0, Point.CARTESIAN),
                    Point(
                        Circle.RADIUS, Circle.RADIUS, Point.CARTESIAN
                    )
                )
            )
            .addPath(
                BezierCurve(
                    Point(Circle.RADIUS, Circle.RADIUS, Point.CARTESIAN),
                    Point(Circle.RADIUS, 2 * Circle.RADIUS, Point.CARTESIAN),
                    Point(0.0, 2 * Circle.RADIUS, Point.CARTESIAN)
                )
            )
            .addPath(
                BezierCurve(
                    Point(0.0, 2 * Circle.RADIUS, Point.CARTESIAN),
                    Point(-Circle.RADIUS, 2 * Circle.RADIUS, Point.CARTESIAN),
                    Point(-Circle.RADIUS, Circle.RADIUS, Point.CARTESIAN)
                )
            )
            .addPath(
                BezierCurve(
                    Point(-Circle.RADIUS, Circle.RADIUS, Point.CARTESIAN),
                    Point(-Circle.RADIUS, 0.0, Point.CARTESIAN),
                    Point(0.0, 0.0, Point.CARTESIAN)
                )
            )
            .build()
        var index = 0
        val spline = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(Pose(0.0,0.0,0.0)),
                    Point(Pose(24.0,0.0,90.0)),
                    Point(Pose(24.0,48.0,0.0)),
                )
            )
            .build()

        val spline2 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(Pose(24.0,48.0,0.0)),
                    Point(Pose(24.0,0.0,90.0)),
                    Point(Pose(0.0,0.0,0.0)),
                )
            )
            .build()
        follower.followPath(spline)
        waitForStart()
        while (opModeIsActive()) {
            follower.update()
            if (follower.atParametricEnd()) {
                index++
                if (index % 2 == 0) {
                    follower.followPath(spline)
                }else{
                    follower.followPath(spline2)
                }
            }
        }
    }
}