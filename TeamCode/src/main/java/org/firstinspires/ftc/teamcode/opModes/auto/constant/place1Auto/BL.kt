package org.firstinspires.ftc.teamcode.opModes.auto.constant.place1Auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Enums.Alliance
import org.firstinspires.ftc.teamcode.Enums.StartSide
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.Companion.getStartPose
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoPatterns.place1Machine
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoPatterns.place1States
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoSorting.place1Sort
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoSorting.preselect
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive
import org.gentrifiedApps.statemachineftc.StateMachine

@Autonomous(group = place1Sort, preselectTeleOp = preselect) //@Disabled

class BL : LinearOpMode() {
    var startPose = autoHardware.startPose
    var robot = autoHardware(this)
    override fun runOpMode() {
        val drive = MecanumDrive(hardwareMap)
        drive.poseEstimate = getStartPose(Alliance.BLUE, StartSide.LEFT)
        val machine: StateMachine<place1States> = place1Machine(drive)
        robot.initAuto(hardwareMap, this, false)
        machine.start()
        while (machine.mainLoop(this)) {
            machine.update()
        }
    }
}