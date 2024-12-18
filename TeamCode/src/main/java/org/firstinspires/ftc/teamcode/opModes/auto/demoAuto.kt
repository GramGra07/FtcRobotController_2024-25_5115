package org.firstinspires.ftc.teamcode.opModes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.customHardware.AutoHardware
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.Point
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles
import org.firstinspires.ftc.teamcode.utilClass.storage.PoseStorage

@Autonomous(group = GroupingTitles.auto) //@Disabled//disabling the opmode
class demoAuto : LinearOpMode() {
    override fun runOpMode() { //if opmode is started
        val robot = AutoHardware(
            this,
            StartLocation(
                Alliance.RED,
                Point(144 - 9.0, 72 + 8.0),
                Math.toRadians(180.0)
            )
        )
        robot.autoSetup()
        robot.once()
        waitForStart()
        robot.localizerSubsystem.follower.followPath(robot.getSpecimenRPath())

//        robot.specimenAutoR.start()
        while (opModeIsActive()) {
//            robot.specimenAutoR.update()
            robot.localizerSubsystem.follower.update()

            PoseStorage.currentPose = robot.localizerSubsystem.pose
        }
    }
}
