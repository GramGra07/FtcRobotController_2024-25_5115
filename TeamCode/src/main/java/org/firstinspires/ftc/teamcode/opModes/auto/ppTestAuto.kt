package org.firstinspires.ftc.teamcode.opModes.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.followers.pedroPathing.follower.Follower
import org.firstinspires.ftc.teamcode.followers.pedroPathing.localization.Pose
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.BezierCurve
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.BezierLine
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.Point
import org.firstinspires.ftc.teamcode.followers.pedroPathing.tuning.Circle
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles


@Autonomous(group = GroupingTitles.auto)
class ppTestAuto : LinearOpMode() {
    override fun runOpMode() {
        val follower = Follower(hardwareMap)

        val telemetryA = MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().telemetry)
        telemetryA.update()
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
                    Point(Pose(0.0, 0.0, 0.0)),
                    Point(Pose(24.0, 0.0, 90.0)),
                    Point(Pose(24.0, 48.0, 0.0)),
                )
            )
            .build()

        val spline2 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(Pose(24.0, 48.0, 0.0)),
                    Point(Pose(24.0, 0.0, 90.0)),
                    Point(Pose(0.0, 0.0, 0.0)),
                )
            )
            .build()
        val fullSpline = follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(Pose(36.0, 36.0, 0.0)),
                    Point(Pose(-36.0, 36.0, 0.0))
                )
            )
            .addPath(
                BezierCurve(
                    Point(Pose(-36.0, 36.0, 90.0)),
                    Point(Pose(-30.0, 24.0, 90.0)),
                    Point(Pose(-44.0, 12.0, 90.0)),
                    Point(Pose(-36.0, 0.0, 90.0)),
                    Point(Pose(-30.0, -12.0, 90.0)),
                    Point(Pose(-44.0, -24.0, 90.0)),
                    Point(Pose(-36.0, -36.0, 180.0)),
                )
            )
            .addPath(
                BezierLine(
                    Point(Pose(-36.0, -36.0, 180.0)),
                    Point(Pose(0.0, -24.0, 180.0))
                ),
            )
            .addPath(
                BezierLine(
                    Point(Pose(0.0, -24.0, 180.0)),
                    Point(Pose(36.0, -36.0, 180.0))
                ),
            )
            .addPath(
                BezierCurve(
                    Point(Pose(36.0, -36.0, 0.0)),
                    Point(Pose(24.0, -36.0, 0.0)),
                    Point(Pose(24.0, 36.0, 270.0)),
                    Point(Pose(36.0, 36.0, 0.0))
                )
            )
            .build()
        follower.followPath(fullSpline)
        follower.setStartingPose(Pose(36.0, 36.0, Math.toRadians(180.0)))
        waitForStart()
        while (opModeIsActive()) {
            follower.update()

            follower.telemetryDebug(telemetryA)
//            if (follower.atParametricEnd()) {
//                index++
//                if (index % 2 == 0) {
//                    follower.followPath(spline)
//                } else {
//                    follower.followPath(spline2)
//                }
//            }
        }
    }
}