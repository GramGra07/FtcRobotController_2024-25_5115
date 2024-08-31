package org.firstinspires.ftc.teamcode.opModes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.followers.pedroPathingOLD.follower.Follower
import org.firstinspires.ftc.teamcode.followers.pedroPathingOLD.localization.Pose
import org.firstinspires.ftc.teamcode.followers.pedroPathingOLD.pathGeneration.BezierCurve
import org.firstinspires.ftc.teamcode.followers.pedroPathingOLD.pathGeneration.Point
import org.firstinspires.ftc.teamcode.followers.pedroPathingOLD.tuning.Circle
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles
import org.gentrifiedApps.statemachineftc.StateMachine

@Autonomous(group = GroupingTitles.auto)
class smppTestAuto : LinearOpMode() {
    enum class states {
        CIRCLE,
        STATE2,
        STOP
    }

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

        val spline = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(Pose(0.0, 0.0, 90.0)),
                    Point(Pose(24.0, 0.0, 90.0)),
                    Point(Pose(24.0, 48.0, 0.0)),
                )
            )
            .build()

        val builder: StateMachine.Builder<states> = StateMachine.Builder<states>()
        builder.state(states.CIRCLE)
            .onEnter(states.CIRCLE) { follower.followPath(spline) }
            .whileState(states.CIRCLE, { follower.atParametricEnd() }, { follower.update() })
            .transition(states.CIRCLE, { follower.atParametricEnd() }, 0.0)
            .stopRunning(states.STOP)
        val stateMachine: StateMachine<states> = builder.build()
        stateMachine.start()
        waitForStart()
        while (stateMachine.mainLoop(this)) {
            stateMachine.update()
        }
    }
}