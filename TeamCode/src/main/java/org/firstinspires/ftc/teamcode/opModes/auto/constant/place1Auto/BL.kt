package org.firstinspires.ftc.teamcode.opModes.auto.constant.place1Auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Enums.Alliance
import org.firstinspires.ftc.teamcode.Enums.StartSide
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.clawSubsystem
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.driveSubsystem
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.AutoHardware
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.AutoHardware.Companion.getStartPose
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoPatterns.place1Machine
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoPatterns.place1States
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoSorting.place1Sort
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoSorting.preselect
import org.gentrifiedApps.statemachineftc.StateMachine

@Autonomous(group = place1Sort, preselectTeleOp = preselect) //@Disabled

class BL : LinearOpMode() {
    var startPose = AutoHardware.startPose
    var robot = AutoHardware(this, hardwareMap, true)
    override fun runOpMode() {
        driveSubsystem!!.drive!!.poseEstimate = getStartPose(Alliance.BLUE, StartSide.LEFT)
        val machine: StateMachine<place1States> = place1Machine(clawSubsystem!!, driveSubsystem!!)
        machine.start()
        while (machine.mainLoop(this)) {
            machine.update()
        }
    }
}