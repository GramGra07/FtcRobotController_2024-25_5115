package org.firstinspires.ftc.teamcode.ggutil.blank

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Enums.EndPose
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.endPose
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage

@Autonomous
@Disabled
class blankAuto : LinearOpMode() {
    var startPose: Pose2d = autoHardware.startPose
    var robot = autoHardware(this)

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val drive = MecanumDrive(hardwareMap)
        drive.poseEstimate = startPose
        robot.initAuto(hardwareMap, this, false)
        if (opModeIsActive()) {
            PoseStorage.currentPose = drive.poseEstimate
        }
        endPose.goToEndPose(EndPose.NONE, drive)
    }
}