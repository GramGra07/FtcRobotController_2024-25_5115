package org.firstinspires.ftc.teamcode.opModes.auto.pixelParkAutos.endOut.insidePath

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Enums.Alliance
import org.firstinspires.ftc.teamcode.Enums.EndPose
import org.firstinspires.ftc.teamcode.Enums.PathLong
import org.firstinspires.ftc.teamcode.Enums.StartSide
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.Companion.getStartPose
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoPatterns.pixelParkMachine
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoPatterns.pixelParkStates
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoSorting.piParkISort
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoSorting.preselect
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive
import org.gentrifiedApps.statemachineftc.StateMachine

@Autonomous(group = piParkISort, preselectTeleOp = preselect) //@Disabled

class PiParkRLIpO : LinearOpMode() {
    var startPose = autoHardware.startPose
    var robot = autoHardware(this)
    override fun runOpMode() {
        val drive = MecanumDrive(hardwareMap)
        drive.poseEstimate = getStartPose(Alliance.RED, StartSide.LEFT)
        val machine: StateMachine<pixelParkStates> =
            pixelParkMachine(drive, PathLong.INSIDE, EndPose.RIGHT)
        robot.initAuto(hardwareMap, this, false)
        machine.start()
        while (machine.mainLoop(this)) {
            machine.update()
        }
    }
}