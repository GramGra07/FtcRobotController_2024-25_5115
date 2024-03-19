package org.firstinspires.ftc.teamcode.ggutil.blank

import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.opModes.PoseStorage
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.AutoHardware
import org.firstinspires.ftc.teamcode.rr.MecanumDrive

@Autonomous
@Disabled
class blankAuto : LinearOpMode() {
    var startPose: Pose2d = AutoHardware.startPose
    var robot = AutoHardware(this, hardwareMap, true)

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val drive = MecanumDrive(hardwareMap, startPose)
        robot.initAuto(hardwareMap, this)
        if (opModeIsActive()) {
            PoseStorage.currentPose = drive.pose
        }
    }
}