package org.firstinspires.ftc.teamcode.utilClass.drivetrain

import org.firstinspires.ftc.teamcode.followers.pedroPathing.localization.Pose
import org.firstinspires.ftc.teamcode.followers.pedroPathing.util.CustomFilteredPIDFCoefficients
import org.firstinspires.ftc.teamcode.followers.pedroPathing.util.CustomPIDFCoefficients
import org.firstinspires.ftc.teamcode.followers.pedroPathing.util.KalmanFilterParameters

data class PedroPathingPARAMS(
    var forwardTicksToInches: Double,
    var lateralTicksToInches: Double,
    var forwardEncoderPose: Pose,
    var lateralEncoderPose: Pose,
    var mass: Double,
    var xMovement: Double,
    var yMovement: Double,
    var forwardZeroPowerAcceleration: Double,
    var lateralZeroPowerAcceleration: Double,
    var translationalPIDFCoefficients: CustomPIDFCoefficients,
    var translationalIntegral: CustomPIDFCoefficients,
    var translationalPIDFFeedForward: Double,
    var headingPIDFCoefficients: CustomPIDFCoefficients,
    var headingPIDFFeedForward: Double,
    var drivePIDFCoefficients: CustomFilteredPIDFCoefficients,
    var drivePIDFFeedForward: Double,
    var driveKalmanFilterParameters: KalmanFilterParameters,
    var zeroPowerAccelerationMultiplier: Double,
    var centripetalScaling: Double,
)
