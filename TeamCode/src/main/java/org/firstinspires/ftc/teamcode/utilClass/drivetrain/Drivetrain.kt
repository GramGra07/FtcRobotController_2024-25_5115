package org.firstinspires.ftc.teamcode.utilClass.drivetrain

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.followers.pedroPathing.localization.Pose
import org.firstinspires.ftc.teamcode.followers.pedroPathing.util.CustomFilteredPIDFCoefficients
import org.firstinspires.ftc.teamcode.followers.pedroPathing.util.CustomPIDFCoefficients
import org.firstinspires.ftc.teamcode.followers.pedroPathing.util.KalmanFilterParameters
import org.firstinspires.ftc.teamcode.storage.CurrentDrivetrain
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.VarConfig

class Drivetrain(
    val name: DrivetrainNames,
    val type: DrivetrainType,
    val rrPARAMS: rrPARAMS,
    var pedroPathingPARAMS: PedroPathingPARAMS,
    var driveConfigPARAMS: DriveConfigPARAMS,
    var sparkFunOTOSParams: SparkFunOTOSParams,
) {
    fun telemetry(telemetry: Telemetry) {
        CurrentDrivetrain.currentDrivetrain.let {
            telemetry.addData("Current Drivetrain", it.name)
            telemetry.addData("Current Drivetrain Type", it.type)
        }
    }

    enum class DrivetrainNames {
        MAIN, // pink one
        OLD,
    }

    companion object {
//        fun drivetrainHasPermission(permission: Permission): Boolean {
//            return CurrentDrivetrain.currentDrivetrain.permissions.contains(permission)
//        }

        var drivetrains: MutableList<Drivetrain> = mutableListOf(
            Drivetrain(
                DrivetrainNames.MAIN,
                DrivetrainType.MECANUM,
                rrPARAMS(
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
                    69.6053,
                    54.3784,
                    -71.9396,
                    -70.1854,
                    CustomPIDFCoefficients(
                        0.04,
                        0.00009,
                        0.002,
                        0.005
                    ),
                    CustomPIDFCoefficients(
                        0.0,
                        0.0,
                        0.0,
                        0.0
                    ),
                    0.015,
                    CustomPIDFCoefficients(
                        2.0,
                        0.0,
                        0.0,
                        0.0
                    ),
                    0.01,
                    CustomFilteredPIDFCoefficients(
                        0.002, //speed
                        0.0005,
                        0.00005,
                        0.0, 0.04
                    ),
                    0.01,
                    KalmanFilterParameters(
                        6.0,
                        1.0
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
                SparkFunOTOSParams(
                    "spark",
                    SparkFunOTOS.Pose2D(
                        VarConfig.OTOSxOffset,
                        VarConfig.OTOSyOffset,
                        VarConfig.OTOShOffset
                    ),
                    VarConfig.linearScalar,
                    1.0105
                ),
            ),
            Drivetrain(
                DrivetrainNames.OLD,
                DrivetrainType.MECANUM,
                rrPARAMS(
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
                    0.015,
                    CustomPIDFCoefficients(
                        2.0,
                        0.0,
                        0.0,
                        0.0
                    ),
                    0.01,
                    CustomFilteredPIDFCoefficients(
                        0.02,
                        0.0,
                        0.000005,
                        0.0, 0.0
                    ),
                    0.01,
                    KalmanFilterParameters(
                        6.0,
                        1.0
                    ),
                    4.0,
                    0.0005
                ),
                DriveConfigPARAMS(
                    "parallelEnc",
                    "lift",
                    LogoFacingDirection.RIGHT,
                    UsbFacingDirection.UP
                ),
                SparkFunOTOSParams(
                    "spark",
                    SparkFunOTOS.Pose2D(
                        VarConfig.OTOSxOffset,
                        VarConfig.OTOSyOffset,
                        VarConfig.OTOShOffset
                    ),
                    1.0,
                    1.0
                ),
            ),
        )
    }
}