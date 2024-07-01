package org.firstinspires.ftc.teamcode.utilClass.drivetrain

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection

data class DriveConfigPARAMS(
    var parEncoder: String,
    var perEncoder: String,
    var logoFacingDirection: LogoFacingDirection,
    var usbFacingDirection: UsbFacingDirection,
)