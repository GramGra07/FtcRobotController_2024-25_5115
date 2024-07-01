package org.firstinspires.ftc.teamcode.opModes.auto

import com.acmerobotics.roadrunner.Pose2d
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
import org.firstinspires.ftc.teamcode.opModes.auto.smppTestAuto.states
import org.gentrifiedApps.statemachineftc.StateMachine

@Autonomous
class smrrTestAuto : LinearOpMode() {
    private lateinit var robot: AutoHardware

    override fun runOpMode() {
        if (isMainDrivetrain()) {
            robot = AutoHardware(
                this, Processor.OBJECT_DETECT, StartLocation(Alliance.RED, StartSide.LEFT)
            )
        } else if (isTesterDrivetrain()) {
            robot = AutoHardware(
                this, null, StartLocation(Alliance.RED, StartSide.LEFT)
            )
        }

        val circle = robot.drive.actionBuilder(robot.drive.pose).splineToSplineHeading(
                Pose2d(10.0, 0.0, Math.toRadians(0.0)), Math.toRadians(90.0)
            ).splineToSplineHeading(Pose2d(10.0, 10.0, Math.toRadians(90.0)), Math.toRadians(180.0))
            .splineToSplineHeading(Pose2d(0.0, 10.0, Math.toRadians(180.0)), Math.toRadians(270.0))
            .splineToSplineHeading(Pose2d(0.0, 0.0, Math.toRadians(270.0)), Math.toRadians(0.0))
            .build()

        val fwd =
            robot.drive.actionBuilder(robot.drive.pose).lineToY(robot.drive.pose.position.y + 10.0)
                .build()

        val builder: StateMachine.Builder<states> = StateMachine.Builder<states>()
        builder.state(states.CIRCLE).onEnter(states.CIRCLE) { runBlocking(circle) }
            .transition(states.CIRCLE, { true }, 0.0).state(states.STATE2)
            .onEnter(states.STATE2) { runBlocking(fwd) }.transition(states.STATE2, { true }, 0.0)
            .stopRunning(states.STOP)
        val stateMachine: StateMachine<states> = builder.build()
        stateMachine.start()

        if (stateMachine.mainLoop(this)) {
            stateMachine.update()
        }
    }
}