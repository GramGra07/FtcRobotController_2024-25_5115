package org.firstinspires.ftc.teamcode.UtilClass

import com.acmerobotics.roadrunner.util.Angle.normDelta
import org.firstinspires.ftc.teamcode.Enums.Alliance
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.IsBusy
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.StartPose
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.updateStatus
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive

object DriverAid {

    fun doDriverAid(
        drive: MecanumDrive,
        goToDrone: Boolean,
        turnStraight: Boolean,
        turnWing: Boolean,
        breakFollowing: Boolean
    ) {
        if (goToDrone) {
            HardwareConfig.slowModeIsOn = false
            IsBusy.isAutoInTeleop = true
            updateStatus("Auto-ing")
            drive.update()
            // go to drone scoring position]
        }
        if (turnStraight) {
            HardwareConfig.slowModeIsOn = false
            IsBusy.isAutoInTeleop = true
            updateStatus("Auto-ing")
            drive.turnAsync(normDelta(Math.toRadians(0.0) - drive.poseEstimate.heading))
        }
        if (turnWing) {
            HardwareConfig.slowModeIsOn = false
            IsBusy.isAutoInTeleop = true
            updateStatus("Auto-ing")
            if (StartPose.alliance == Alliance.RED) {
                drive.turnAsync(normDelta(Math.toRadians(135.0) - drive.poseEstimate.heading))
            } else {
                drive.turnAsync(normDelta(Math.toRadians(-135.0) - drive.poseEstimate.heading))
            }
        }
        if (!drive.isBusy) {
            IsBusy.isAutoInTeleop = false
            updateStatus("Running")
        }
        if (breakFollowing) {
            IsBusy.isAutoInTeleop = false
            updateStatus("Running")
            drive.breakFollowing()
        }
    }
}
