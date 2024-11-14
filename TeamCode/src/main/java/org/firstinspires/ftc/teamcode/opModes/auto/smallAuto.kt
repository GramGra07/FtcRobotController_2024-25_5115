package org.firstinspires.ftc.teamcode.opModes.auto

import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation.Companion.ofContext
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles

@Autonomous(group = GroupingTitles.tele) //@Disabled//disabling the opmode
//@Disabled
class smallAuto : LinearOpMode() {


    //declaring the class
    override fun runOpMode() { //if opmode is started
        val robot = HardwareConfig(
            this,
            true,
            StartLocation(Alliance.RED, Pose2d.ofContext(StartLocation.SPOTS.RL))
        )
        robot.scoringSubsystem.setup()
        //0.5 power
        val veloInSHalf = 20
        fun travel(distance: Double) {
            robot.driveSubsystem.setArtificialPower(-0.5, 0.0)
            sleep(((distance / veloInSHalf) * 1000).toLong())
            robot.driveSubsystem.setArtificialPower(0.0, 0.0)
        }
        waitForStart()
//        robot.once() //runs once
        if (opModeIsActive()) { //while the op mode is active
            robot.scoringSubsystem.setup()
            travel(30.0)
        }
    }
}
