package org.firstinspires.ftc.teamcode.opModes.auto.demos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.followers.pedroPathing.follower.Follower
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.BezierLine
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.BezierPoint
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.Path
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.Point
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles

@Autonomous(group = GroupingTitles.demos)
class stayDemoPP : OpMode() {

    private var follower: Follower? = null

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    override fun init() {
        follower = Follower(hardwareMap)
    }

    override fun loop() {
        follower!!.holdPoint(BezierPoint(Point(0.0,0.0,0)),Math.toRadians(90.0))
    }
}