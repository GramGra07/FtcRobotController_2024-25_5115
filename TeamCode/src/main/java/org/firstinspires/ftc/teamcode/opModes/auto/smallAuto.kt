package org.firstinspires.ftc.teamcode.opModes.auto

import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation.Companion.ofContext
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance
import org.firstinspires.ftc.teamcode.extensions.PoseExtensions.toPose
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.BezierCurve
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.Path
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.Point
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles
import org.gentrifiedApps.statemachineftc.StateMachine

@Autonomous(group = GroupingTitles.tele) //@Disabled//disabling the opmode
@Disabled
class smallAuto : LinearOpMode() {
    enum class STATES {
        STATE1, STATE2, STATE3, STOP
    }

    //declaring the class
    override fun runOpMode() { //if opmode is started
        val robot = HardwareConfig(
            this,
            true,
            StartLocation(Alliance.RED, Pose2d.ofContext(StartLocation.SPOTS.RL))
        )

        val sm = StateMachine.Builder<STATES>()
            .state(STATES.STATE1)
            .onEnter(STATES.STATE1) {
                robot.driveFollower.followPath(
                    Path(
                        BezierCurve(
                            Point(robot.localizerSubsystem.start().toPose()), Point(
                                60.0, 60.0,
                                Math.toRadians(90.0).toInt()
                            )
                        )
                    )
                )
            }
            .onExit(STATES.STATE1) {}
            .transition(STATES.STOP, { robot.driveFollower.atParametricEnd() }, 0.0)
            .stopRunning(STATES.STOP)
            .build()
        sm.start()
        waitForStart()
        robot.once() //runs once
        while (sm.mainLoop(this)) { //while the op mode is active
            sm.update()
        }
    }
}
