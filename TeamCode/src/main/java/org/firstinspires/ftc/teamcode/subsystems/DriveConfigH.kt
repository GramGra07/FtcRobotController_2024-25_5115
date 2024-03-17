package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot

@Config
class DriveConfigH {
    companion object {
        val TICKS_PER_REV = 28.0
        val MAX_RPM = 400.0
        var WHEEL_RADIUS = 1.88 // in (96mm)

        var GEAR_RATIO = 1 / 15.0 // output (wheel) speed / input (motor) speed

        var TRACK_WIDTH = 23.19 // in
        var USB_FACING_DIR: RevHubOrientationOnRobot.UsbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;
        var LOGO_FACING_DIR: RevHubOrientationOnRobot.LogoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
    }
}