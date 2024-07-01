package org.firstinspires.ftc.teamcode.opModes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.followers.pedroPathing.follower.Follower
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.BezierCurve
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.Point
import org.firstinspires.ftc.teamcode.followers.pedroPathing.tuning.Circle
import org.gentrifiedApps.statemachineftc.StateMachine

@Autonomous
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
                    Point(0.0, 0.0, Point.CARTESIAN),
                    Point(Circle.RADIUS, 0.0, Point.CARTESIAN),
                    Point(
                        Circle.RADIUS, Circle.RADIUS, Point.CARTESIAN
                    )
                )
            )
            .build()

        val builder: StateMachine.Builder<states> = StateMachine.Builder<states>()
        builder.state(states.CIRCLE)
            .onEnter(states.CIRCLE) { follower.followPath(circle) }
            .whileState(states.CIRCLE ,{follower.atParametricEnd() },{follower.update()})
            .transition(states.CIRCLE,{follower.atParametricEnd()},0.0)
            .state(states.STATE2)
            .onEnter(states.STATE2) { follower.followPath(spline) }
            .whileState(states.STATE2 ,{follower.atParametricEnd() },{follower.update()})
            .transition(states.STATE2,{follower.atParametricEnd()},0.0)
            .stopRunning(states.STOP)
        val stateMachine: StateMachine<states> = builder.build()
        stateMachine.start()

        while (stateMachine.mainLoop(this)) {
            stateMachine.update()
//            follower.update()
//            if (follower.atParametricEnd()) {
//                follower.followPath(circle)
//            }
        }
    }
}