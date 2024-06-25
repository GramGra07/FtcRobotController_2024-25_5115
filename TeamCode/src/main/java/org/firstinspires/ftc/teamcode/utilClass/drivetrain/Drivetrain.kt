package org.firstinspires.ftc.teamcode.utilClass.drivetrain

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.storage.CurrentDrivetrain

class Drivetrain(
    val name: DrivetrainNames,
    val type: DrivetrainType,
    val reversedMotors: MutableList<motorNames>,
    val drivePARAMS: DrivePARAMS,
    val localizerPARAMS: LocalizerPARAMS,
    var logoFacingDirection: LogoFacingDirection,
    var usbFacingDirection: UsbFacingDirection,
) {
    fun telemetry(telemetry: Telemetry) {

        CurrentDrivetrain.currentDrivetrain.let {
            telemetry.addData("Current Drivetrain", it.name)
            telemetry.addData("Current Drivetrain Type", it.type)
        }
    }

    enum class DrivetrainNames {
        TESTER, // stripped one
        MAIN, // pink one
        SECONDARY
    }

    enum class motorNames {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    }

    companion object {

        var drivetrains: MutableList<Drivetrain> = mutableListOf(
            Drivetrain(
                DrivetrainNames.TESTER,
                DrivetrainType.MECANUM,
                mutableListOf(motorNames.BACK_LEFT, motorNames.FRONT_LEFT),
                DrivePARAMS(
                    0.0029780281585,
                    0.0019808842928630615,
                    4581.065265532445,
                    1.4685147318880003,
                    0.0004545805461853856,
                    0.000088,
                    50.0,
                    -30.0,
                    50.0,
                    Math.PI,
                    Math.PI,
                    5.0,
                    3.0,
                    1.0,
                    0.0,
                    0.0,
                    0.0
                ),
                LocalizerPARAMS(
                    -2187.424701924342,
                    -241.8308218367524
                ),
                LogoFacingDirection.RIGHT,
                UsbFacingDirection.UP,
            ),
            Drivetrain(
                DrivetrainNames.MAIN,
                DrivetrainType.MECANUM,
                mutableListOf(motorNames.FRONT_RIGHT),
                DrivePARAMS(
                    0.0029780281585,
                    0.0019808842928630615,
                    4581.065265532445,
                    1.4685147318880003,
                    0.0004545805461853856,
                    0.000088,
                    50.0,
                    -30.0,
                    50.0,
                    Math.PI,
                    Math.PI,
                    5.0,
                    3.0,
                    1.0,
                    0.0,
                    0.0,
                    0.0
                ),
                LocalizerPARAMS(
                    -2187.424701924342,
                    -241.8308218367524
                ),
                LogoFacingDirection.RIGHT,
                UsbFacingDirection.UP,
            ),
        )
    }
}