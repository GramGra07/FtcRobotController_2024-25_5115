package org.firstinspires.ftc.teamcode.UtilClass

import com.acmerobotics.roadrunner.util.Angle.normDelta
import org.firstinspires.ftc.teamcode.Enums.Alliance
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.StartPose
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.updateStatus
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem

object DriverAid {

    fun doDriverAid(
        driveSubsystem: DriveSubsystem,
        goToDrone: Boolean,
        turnStraight: Boolean,
        turnWing: Boolean,
        breakFollowing: Boolean,
    ) {
        val drive: MecanumDrive = driveSubsystem.drive!!
        if (goToDrone) {
            driveSubsystem.slowModeIsOn = false
            driveSubsystem.isAutoInTeleop = true
            updateStatus("Auto-ing")
            drive.update()
            // go to drone scoring position]
        }
        if (turnStraight) {
            driveSubsystem.slowModeIsOn = false
            driveSubsystem.isAutoInTeleop = true
            updateStatus("Auto-ing")
            drive.turnAsync(normDelta(Math.toRadians(0.0) - drive.poseEstimate.heading))
        }
        if (turnWing) {
            driveSubsystem.slowModeIsOn = false
            driveSubsystem.isAutoInTeleop = true
            updateStatus("Auto-ing")
            if (StartPose.alliance == Alliance.RED) {
                drive.turnAsync(normDelta(Math.toRadians(135.0) - drive.poseEstimate.heading))
            } else {
                drive.turnAsync(normDelta(Math.toRadians(-135.0) - drive.poseEstimate.heading))
            }
        }
        if (!drive.isBusy) {
            driveSubsystem.isAutoInTeleop = false
            updateStatus("Running")
        }
        if (breakFollowing) {
            driveSubsystem.isAutoInTeleop = false
            updateStatus("Running")
            drive.breakFollowing()
        }
    }
}
