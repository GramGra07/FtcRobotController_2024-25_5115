package org.firstinspires.ftc.teamcode.utilClass.drivetrain

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.followers.pedroPathing.localization.Pose
import org.firstinspires.ftc.teamcode.followers.pedroPathing.util.CustomPIDFCoefficients
import org.firstinspires.ftc.teamcode.storage.CurrentDrivetrain
import org.firstinspires.ftc.teamcode.utilClass.objects.Permission

class Drivetrain(
    val name: DrivetrainNames,
    val type: DrivetrainType,
    val drivePARAMS: DrivePARAMS,
    val localizerPARAMS: LocalizerPARAMS,
    var pedroPathingPARAMS: PedroPathingPARAMS,
    var driveConfigPARAMS: DriveConfigPARAMS,
    var permissions: MutableList<Permission>,
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
        fun drivetrainHasPermission(permission: Permission): Boolean {
            return CurrentDrivetrain.currentDrivetrain.permissions.contains(permission)
        }

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
                PedroPathingPARAMS(
                    0.002550207204,
                    -0.0020726061118353043,
                    Pose(0.0, 0.0, 0.0),
                    Pose(0.0, 0.0, Math.toRadians(90.0)),
                    10.0,
                    81.0,
                    65.0,
                    -34.62719,
                    -78.15554,
                    CustomPIDFCoefficients(
                        0.3,
                        0.0,
                        0.01,
                        0.0
                    ),
                    CustomPIDFCoefficients(
                        0.1,
                        0.0,
                        0.0,
                        0.0
                    ),
                    CustomPIDFCoefficients(
                        1.0,
                        0.0,
                        0.0,
                        0.0
                    ),
                    CustomPIDFCoefficients(
                        5.0,
                        0.0,
                        0.08,
                        0.0
                    ),
                    4.0,
                    CustomPIDFCoefficients(
                        0.02,
                        0.0,
                        0.000005,
                        0.0
                    ),
                    CustomPIDFCoefficients(
                        0.025,
                        0.0,
                        0.00001,
                        0.0
                    ),
                    0.0005
                ),
                DriveConfigPARAMS(
                    "motorFrontLeft",
                    "motorBackLeft",
                    LogoFacingDirection.UP,
                    UsbFacingDirection.LEFT
                ),
                mutableListOf(),
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
                PedroPathingPARAMS(
                    0.002550207204,
                    -0.0020726061118353043,
                    Pose(0.0, 0.0, 0.0),
                    Pose(0.0, 0.0, Math.toRadians(90.0)),
                    10.0,
                    81.0,
                    65.0,
                    -34.62719,
                    -78.15554,
                    CustomPIDFCoefficients(
                        0.3,
                        0.0,
                        0.01,
                        0.0
                    ),
                    CustomPIDFCoefficients(
                        0.1,
                        0.0,
                        0.0,
                        0.0
                    ),
                    CustomPIDFCoefficients(
                        1.0,
                        0.0,
                        0.0,
                        0.0
                    ),
                    CustomPIDFCoefficients(
                        5.0,
                        0.0,
                        0.08,
                        0.0
                    ),
                    4.0,
                    CustomPIDFCoefficients(
                        0.02,
                        0.0,
                        0.000005,
                        0.0
                    ),
                    CustomPIDFCoefficients(
                        0.025,
                        0.0,
                        0.00001,
                        0.0
                    ),
                    0.0005,
                ),
                DriveConfigPARAMS(
                    "parallelEnc",
                    "lift",
                    LogoFacingDirection.RIGHT,
                    UsbFacingDirection.UP
                ),
                mutableListOf(
                    Permission.EXTRAS,
                    Permission.EXTENDO,
                    Permission.CLAW,
                    Permission.ENDGAME,
                    Permission.LOCALIZATION,
                    Permission.LIGHTS
                ),
            ),
        )
    }
}