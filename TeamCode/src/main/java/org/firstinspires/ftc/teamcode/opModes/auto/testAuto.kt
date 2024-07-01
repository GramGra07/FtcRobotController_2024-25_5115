package org.firstinspires.ftc.teamcode.opModes.auto

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.customHardware.AutoHardware
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig.Companion.isMainDrivetrain
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig.Companion.isTesterDrivetrain
import org.firstinspires.ftc.teamcode.customHardware.autoHardware.StartLocation
import org.firstinspires.ftc.teamcode.customHardware.camera.camUtil.Processor
import org.firstinspires.ftc.teamcode.startEnums.Alliance
import org.firstinspires.ftc.teamcode.startEnums.StartSide


@Autonomous
class testAuto : LinearOpMode() {
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
        if (opModeIsActive()) {
            var trajectoryAction1 = robot.drive.actionBuilder(robot.drive.pose)
                .lineToYSplineHeading(33.0, Math.toRadians(0.0))
                .waitSeconds(2.0)
                .setTangent(Math.toRadians(90.0))
                .lineToY(48.0)
                .setTangent(Math.toRadians(0.0))
                .lineToX(32.0)
                .strafeTo(Vector2d(44.5, 30.0))
                .turn(Math.toRadians(180.0))
                .lineToX(47.5)
                .waitSeconds(3.0)
                .build()
            Actions.runBlocking(
                SequentialAction(
                    trajectoryAction1,
                )
            )
        }
    }
}