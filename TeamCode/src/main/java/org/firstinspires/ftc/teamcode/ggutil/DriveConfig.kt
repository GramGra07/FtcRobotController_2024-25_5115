package org.firstinspires.ftc.teamcode.ggutil

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.PIDFCoefficients


@Config
class DriveConfig private constructor() {
    companion object {
        @JvmField
        val TICKS_PER_REV = 28.0

        @JvmField
        val MAX_RPM = 400.0

        @JvmField
        var WHEEL_RADIUS = 1.88 // in (96mm)

        @JvmField
        var GEAR_RATIO = 1 / 15.0 // output (wheel) speed / input (motor) speed

        @JvmField
        var TRACK_WIDTH = 23.19 // in
        var USB_FACING_DIR: RevHubOrientationOnRobot.UsbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.UP
        var LOGO_FACING_DIR: RevHubOrientationOnRobot.LogoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD
        const val RUN_USING_ENCODER = true

        @JvmField
        var MOTOR_VELO_PID = PIDFCoefficients(
            0.0, 0.0, 0.0,
            12.7
        )

        @JvmField
        var kV = 0.004

        @JvmField
        var kA = 0.0002

        @JvmField
        var kStatic = 0.005

        @JvmField
        var MAX_VEL = 55.0

        @JvmField
        var MAX_ACCEL = 60.0

        @JvmField
        var MAX_ANG_VEL = 3.0935962200164795

        @JvmField
        var MAX_ANG_ACCEL = Math.toRadians(360.0)


        fun encoderTicksToInches(ticks: Double): Double {
            return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV
        }

        fun rpmToVelocity(rpm: Double): Double {
            return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0
        }

        fun getMotorVelocityF(ticksPerSecond: Double): Double {
            // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
            return 32767 / ticksPerSecond
        }
    }
}