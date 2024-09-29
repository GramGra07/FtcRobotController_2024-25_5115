package org.firstinspires.ftc.teamcode.followers.pedroPathing.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.followers.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles;

/**
 * This is the Circle autonomous OpMode. It runs the robot in a PathChain that's actually not quite
 * a circle, but some Bezier curves that have control points set essentially in a square. However,
 * it turns enough to tune your centripetal force correction and some of your heading. Some lag in
 * heading is to be expected.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@Autonomous(name = "Circle", group = GroupingTitles.pedroTuning)
@Disabled
public class Circle extends OpMode {
    public static double RADIUS = 10;
    private Telemetry telemetryA;
    private Follower follower;

    private PathChain circle;

    /**
     * This initializes the Follower and creates the PathChain for the "circle". Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap);

        circle = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(0, 0, Point.CARTESIAN), new Point(RADIUS, 0, Point.CARTESIAN), new Point(RADIUS, RADIUS, Point.CARTESIAN)))
                .addPath(new BezierCurve(new Point(RADIUS, RADIUS, Point.CARTESIAN), new Point(RADIUS, 2 * RADIUS, Point.CARTESIAN), new Point(0, 2 * RADIUS, Point.CARTESIAN)))
                .addPath(new BezierCurve(new Point(0, 2 * RADIUS, Point.CARTESIAN), new Point(-RADIUS, 2 * RADIUS, Point.CARTESIAN), new Point(-RADIUS, RADIUS, Point.CARTESIAN)))
                .addPath(new BezierCurve(new Point(-RADIUS, RADIUS, Point.CARTESIAN), new Point(-RADIUS, 0, Point.CARTESIAN), new Point(0, 0, Point.CARTESIAN)))
                .build();

        follower.followPath(circle);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run in a roughly circular shape of radius " + RADIUS
                + ", starting on the right-most edge. So, make sure you have enough "
                + "space to the left, front, and back to run the OpMode.");
        telemetryA.update();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();
        if (follower.atParametricEnd()) {
            follower.followPath(circle);
        }

        follower.telemetryDebug(telemetryA);
    }
}
