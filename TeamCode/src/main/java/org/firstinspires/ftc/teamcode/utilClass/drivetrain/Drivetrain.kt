package org.firstinspires.ftc.teamcode.utilClass.drivetrain

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.storage.CurrentDrivetrain

class Drivetrain(
    val name: DrivetrainNames,
    val type: DrivetrainType,
    val drivePARAMS: DrivePARAMS,
    val localizerPARAMS: LocalizerPARAMS,
    var logoFacingDirection: LogoFacingDirection,
    var usbFacingDirection: UsbFacingDirection,
    var parEncoder: String,
    var perEncoder: String,
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

    companion object {

        var drivetrains: MutableList<Drivetrain> = mutableListOf(
            Drivetrain(
                DrivetrainNames.TESTER,
                DrivetrainType.MECANUM,
                DrivePARAMS(
                    0.002550207204,
                    -0.0020726061118353043,
                    -4768.043260040023,
                    0.8666821746445845,
                    0.0004995225996863677,
                    0.0001,
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
                    724.8676841202279,
                    -1689.0290452639733
                ),
                LogoFacingDirection.UP,
                UsbFacingDirection.LEFT,
                "motorFrontLeft",
                "motorBackLeft"
            ),
            Drivetrain(
                DrivetrainNames.MAIN,
                DrivetrainType.MECANUM,
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
                "parallelEnc",
                "lift"
            ),
        )
    }
}