package org.firstinspires.ftc.teamcode.opModes.auto.demos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.followers.pedroPathing.follower.Follower
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.BezierPoint
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.Point
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles

@Autonomous(group = GroupingTitles.demos)
//@Disabled
class stayDemoPP : LinearOpMode() {

    private var follower: Follower? = null

    override fun runOpMode() {
        follower = Follower(hardwareMap)
        waitForStart()
        while (opModeIsActive()) {
            follower!!.holdPoint(BezierPoint(Point(0.0, 0.0, 0)), Math.toRadians(90.0))
        }
    }
}