package org.firstinspires.ftc.teamcode.utilClass.drivetrain

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.followers.pedroPathing.localization.Pose
import org.firstinspires.ftc.teamcode.followers.pedroPathing.util.CustomFilteredPIDFCoefficients
import org.firstinspires.ftc.teamcode.followers.pedroPathing.util.CustomPIDFCoefficients
import org.firstinspires.ftc.teamcode.storage.CurrentDrivetrain
import org.firstinspires.ftc.teamcode.utilClass.objects.Permission

class Drivetrain(
    val name: DrivetrainNames,
    val type: DrivetrainType,
    val drivePARAMS: DrivePARAMS,
    var pedroPathingPARAMS: PedroPathingPARAMS,
    var driveConfigPARAMS: DriveConfigPARAMS,
    var sparkFunOTOSParams: SparkFunOTOSParams,
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
                    783.0500500149055,
                    1678.229840435962,
                    0.002550207204,
                    -0.0020726061118353043,
                    4763.191550037603,
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
                PedroPathingPARAMS(
                    0.00295188428614,
                    -0.002490105300198796,
                    Pose(0.0, 809.2789524675853, 0.0), //fwd //parallel
                    Pose(1745.8006924415185, 0.0, 0.0), //lateral //perp
                    6.1,
                    60.758503953976685,
                    40.8115164940066,
                    -50.12846614166424,
                    -77.35699334482771,
                    CustomPIDFCoefficients(
                        0.0001,
                        0.00006,
                        0.005,
                        0.0005
                    ),
                    CustomPIDFCoefficients(
                        0.09,
                        0.0,
                        0.001,
                        0.0
                    ),
                    CustomPIDFCoefficients(
                        2.0,
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
                    CustomFilteredPIDFCoefficients(
                        0.02,
                        0.0,
                        0.000005,
                        0.0,0.0
                    ),
                    CustomFilteredPIDFCoefficients(
                        0.025,
                        0.0,
                        0.00001,
                        0.0,0.0
                    ),
                    4.0,
                    0.0005
                ),
                DriveConfigPARAMS(
                    "motorFrontLeft",
                    "motorBackLeft",
                    LogoFacingDirection.UP,
                    UsbFacingDirection.LEFT
                ),
                SparkFunOTOSParams("spark", SparkFunOTOS.Pose2D(1.5, -7.5, 0.0),1.0,1.0),
                mutableListOf(Permission.RELOCALIZATION),
            ),
            Drivetrain(
                DrivetrainNames.MAIN,
                DrivetrainType.MECANUM,
                DrivePARAMS(
                    -2187.424701924342,
                    -241.8308218367524,
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
                PedroPathingPARAMS(
                    0.000750207204,
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
                        5.0,
                        0.0,
                        0.08,
                        0.0
                    ),
                    CustomPIDFCoefficients(
                        1.0,
                        0.0,
                        0.0,
                        0.0
                    ),
                    CustomFilteredPIDFCoefficients(
                        0.02,
                        0.0,
                        0.000005,
                        0.0,0.0
                    ),
                    CustomFilteredPIDFCoefficients(
                        0.025,
                        0.0,
                        0.00001,
                        0.0,0.0
                    ),
                    4.0,
                    0.0005,
                ),
                DriveConfigPARAMS(
                    "parallelEnc",
                    "lift",
                    LogoFacingDirection.RIGHT,
                    UsbFacingDirection.UP
                ),
                SparkFunOTOSParams("spark", SparkFunOTOS.Pose2D(1.5, -7.5, 0.0),1.0,1.0),
                mutableListOf(
                    Permission.EXTRAS,
                    Permission.EXTENDO,
                    Permission.CLAW,
                    Permission.ENDGAME,
                    Permission.RELOCALIZATION,
                    Permission.LIGHTS
                ),
            ),
        )
    }
}