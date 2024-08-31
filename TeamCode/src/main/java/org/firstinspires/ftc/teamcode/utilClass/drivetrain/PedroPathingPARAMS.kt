package org.firstinspires.ftc.teamcode.utilClass.drivetrain

import org.firstinspires.ftc.teamcode.followers.pedroPathingOLD.localization.Pose
import org.firstinspires.ftc.teamcode.followers.pedroPathingOLD.util.CustomFilteredPIDFCoefficients
import org.firstinspires.ftc.teamcode.followers.pedroPathingOLD.util.CustomPIDFCoefficients

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
    var smallTranslationalPIDFCoefficients: CustomPIDFCoefficients,
    var largeTranslationalPIDFCoefficients: CustomPIDFCoefficients,
    var smallHeadingPIDFCoefficients: CustomPIDFCoefficients,
    var largeHeadingPIDFCoefficients: CustomPIDFCoefficients,
    var smallDrivePIDFCoefficients: CustomFilteredPIDFCoefficients,
    var largeDrivePIDFCoefficients: CustomFilteredPIDFCoefficients,
    var zeroPowerAccelerationMultiplier: Double,
    var centripetalScaling: Double,
)
