package org.firstinspires.ftc.teamcode.opModes.auto.demos

import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.customHardware.AutoHardware
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig.Companion.isMainDrivetrain
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig.Companion.isTesterDrivetrain
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.StartSide
import org.firstinspires.ftc.teamcode.customHardware.camera.camUtil.Processor
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.Vector

@Autonomous
class stayDemoRR : LinearOpMode() {
    private lateinit var robot: AutoHardware

    override fun runOpMode() {
        if (isMainDrivetrain()) {
            robot =
                AutoHardware(
                    this,
                    Processor.OBJECT_DETECT,
                    StartLocation(Alliance.RED, StartSide.LEFT)
                )
        } else if (isTesterDrivetrain()) {
            robot =
                AutoHardware(
                    this,
                    null,
                    StartLocation(Alliance.RED, StartSide.LEFT)
                )
        }
        val startX = robot.drive.pose.position.x
        val startY = robot.drive.pose.position.y
        val startHeading = robot.drive.pose.heading
        val traj = robot.drive.actionBuilder(robot.drive.pose)
            .splineTo(Vector2d(startX,startY),startHeading)
            .build()
        while (opModeIsActive()) {
            runBlocking(
                traj
            )
        }
    }
}