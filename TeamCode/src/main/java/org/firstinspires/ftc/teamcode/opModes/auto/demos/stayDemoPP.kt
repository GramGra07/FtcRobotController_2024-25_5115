package org.firstinspires.ftc.teamcode.opModes.auto.demos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.followers.pedroPathingOLD.follower.Follower
import org.firstinspires.ftc.teamcode.followers.pedroPathingOLD.pathGeneration.BezierLine
import org.firstinspires.ftc.teamcode.followers.pedroPathingOLD.pathGeneration.Path
import org.firstinspires.ftc.teamcode.followers.pedroPathingOLD.pathGeneration.Point
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles

@Autonomous(group = GroupingTitles.demos)
class stayDemoPP : OpMode() {
    var DISTANCE: Double = 40.0
    private var forward = true

    private var follower: Follower? = null

    private var forwards: Path? = null
    private var backwards: Path? = null

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    override fun init() {
        follower = Follower(hardwareMap)
        Follower.useDrive = false

        forwards = Path(
            BezierLine(
                Point(0.0, 0.0, Point.CARTESIAN),
                Point(DISTANCE, 0.0, Point.CARTESIAN)
            )
        )
        forwards!!.setConstantHeadingInterpolation(0.0)
        backwards = Path(
            BezierLine(
                Point(DISTANCE, 0.0, Point.CARTESIAN),
                Point(0.0, 0.0, Point.CARTESIAN)
            )
        )
        backwards!!.setConstantHeadingInterpolation(0.0)

        follower!!.followPath(forwards)
    }

    override fun loop() {
        follower!!.update()
        if (!follower!!.isBusy) {
            if (forward) {
                forward = false
                follower!!.followPath(backwards)
            } else {
                forward = true
                follower!!.followPath(forwards)
            }
        }
    }
}