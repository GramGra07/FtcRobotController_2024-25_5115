package org.firstinspires.ftc.teamcode.followers.pedroPathing.tuning;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.followers.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.storage.CurrentDrivetrain;

/**
 * This is the FollowerConstants class. It holds many constants and parameters for various parts of
 * the Follower. This is here to allow for easier tuning of Pedro Pathing, as well as concentrate
 * everything tunable for the Paths themselves in one place.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/4/2024
 */
@Config
public class FollowerConstants {

    // Large translational PIDF coefficients
    public static CustomPIDFCoefficients largeTranslationalPIDFCoefficients = CurrentDrivetrain.Companion.getCurrentDrivetrain().getPedroPathingPARAMS().getLargeTranslationalPIDFCoefficients();
    // Feed forward constant added on to the large translational PIDF
    public static double largeTranslationalPIDFFeedForward = 0.015;
    // Large translational Integral
    public static CustomPIDFCoefficients largeTranslationalIntegral = new CustomPIDFCoefficients(
            0,
            0,
            0,
            0);
    // the limit at which the heading PIDF switches between the large and small translational PIDFs
    public static double translationalPIDFSwitch = 3;
    // Small translational PIDF coefficients
    public static CustomPIDFCoefficients smallTranslationalPIDFCoefficients = CurrentDrivetrain.Companion.getCurrentDrivetrain().getPedroPathingPARAMS().getSmallTranslationalPIDFCoefficients();
    // Small translational Integral value
    public static CustomPIDFCoefficients smallTranslationalIntegral = new CustomPIDFCoefficients(
            0,
            0,
            0,
            0);
    // Feed forward constant added on to the small translational PIDF
    public static double smallTranslationalPIDFFeedForward = 0.015;
    // Large heading error PIDF coefficients
    public static CustomPIDFCoefficients largeHeadingPIDFCoefficients = CurrentDrivetrain.Companion.getCurrentDrivetrain().getPedroPathingPARAMS().getLargeHeadingPIDFCoefficients();
    // Feed forward constant added on to the large heading PIDF
    public static double largeHeadingPIDFFeedForward = 0.01;
    // the limit at which the heading PIDF switches between the large and small heading PIDFs
    public static double headingPIDFSwitch = Math.PI / 20;
    // Small heading error PIDF coefficients
    public static CustomPIDFCoefficients smallHeadingPIDFCoefficients = CurrentDrivetrain.Companion.getCurrentDrivetrain().getPedroPathingPARAMS().getSmallHeadingPIDFCoefficients();
    // Feed forward constant added on to the small heading PIDF
    public static double smallHeadingPIDFFeedForward = 0.01;
    // Large drive PIDF coefficients
    public static CustomPIDFCoefficients largeDrivePIDFCoefficients = CurrentDrivetrain.Companion.getCurrentDrivetrain().getPedroPathingPARAMS().getLargeDrivePIDFCoefficients();
    // Feed forward constant added on to the large drive PIDF
    public static double largeDrivePIDFFeedForward = 0.01;
    // the limit at which the heading PIDF switches between the large and small drive PIDFs
    public static double drivePIDFSwitch = 20;
    // Small drive PIDF coefficients
    public static CustomPIDFCoefficients smallDrivePIDFCoefficients = CurrentDrivetrain.Companion.getCurrentDrivetrain().getPedroPathingPARAMS().getSmallDrivePIDFCoefficients();
    // Feed forward constant added on to the small drive PIDF
    public static double smallDrivePIDFFeedForward = 0.01;
    // Mass of robot in kilograms
    public static double mass = CurrentDrivetrain.Companion.getCurrentDrivetrain().getPedroPathingPARAMS().getMass();
    // Centripetal force to power scaling
    public static double centripetalScaling = CurrentDrivetrain.Companion.getCurrentDrivetrain().getPedroPathingPARAMS().getCentripetalScaling();
    // Acceleration of the drivetrain when power is cut in inches/second^2 (should be negative)
    // if not negative, then the robot thinks that its going to go faster under 0 power
    public static double forwardZeroPowerAcceleration = CurrentDrivetrain.Companion.getCurrentDrivetrain().getPedroPathingPARAMS().getForwardZeroPowerAcceleration();
    // Acceleration of the drivetrain when power is cut in inches/second^2 (should be negative)
    // if not negative, then the robot thinks that its going to go faster under 0 power
    public static double lateralZeroPowerAcceleration = CurrentDrivetrain.Companion.getCurrentDrivetrain().getPedroPathingPARAMS().getLateralZeroPowerAcceleration();
    // A multiplier for the zero power acceleration to change the speed the robot decelerates at
    // the end of paths.
    // Increasing this will cause the robot to try to decelerate faster, at the risk of overshoots
    // or localization slippage.
    // Decreasing this will cause the deceleration at the end of the Path to be slower, making the
    // robot slower but reducing risk of end-of-path overshoots or localization slippage.
    // This can be set individually for each Path, but this is the default.
    public static double zeroPowerAccelerationMultiplier = CurrentDrivetrain.Companion.getCurrentDrivetrain().getPedroPathingPARAMS().getZeroPowerAccelerationMultiplier();
    // When the robot is at the end of its current Path or PathChain and the velocity goes below
    // this value, then end the Path. This is in inches/second.
    // This can be custom set for each Path.
    public static double pathEndVelocityConstraint = 0.1;
    // When the robot is at the end of its current Path or PathChain and the translational error
    // goes below this value, then end the Path. This is in inches.
    // This can be custom set for each Path.
    public static double pathEndTranslationalConstraint = 0.1;
    // When the robot is at the end of its current Path or PathChain and the heading error goes
    // below this value, then end the Path. This is in radians.
    // This can be custom set for each Path.
    public static double pathEndHeadingConstraint = 0.007;
    // When the t-value of the closest point to the robot on the Path is greater than this value,
    // then the Path is considered at its end.
    // This can be custom set for each Path.
    public static double pathEndTValueConstraint = 0.995;
    // When the Path is considered at its end parametrically, then the Follower has this many
    // milliseconds to further correct by default.
    // This can be custom set for each Path.
    public static double pathEndTimeoutConstraint = 500;
    // This is how many steps the BezierCurve class uses to approximate the length of a BezierCurve.
    public static int APPROXIMATION_STEPS = 1000;
    // This is scales the translational error correction power when the Follower is holding a Point.
    public static double holdPointTranslationalScaling = 0.45;
    // This is scales the heading error correction power when the Follower is holding a Point.
    public static double holdPointHeadingScaling = 0.35;
    // This is the number of times the velocity is recorded for averaging when approximating a first
    // and second derivative for on the fly centripetal correction. The velocity is calculated using
    // half of this number of samples, and the acceleration uses all of this number of samples.
    public static int AVERAGED_VELOCITY_SAMPLE_NUMBER = 8;
    // This is the number of steps the binary search for closest point uses. More steps is more
    // accuracy, and this increases at an exponential rate. However, more steps also does take more
    // time.
    public static int BEZIER_CURVE_BINARY_STEP_LIMIT = 10;
    // This section is for setting the actual drive vector for the front left wheel, if the robot
    // is facing a heading of 0 radians with the wheel centered at (0,0)
    private static final double xMovement = CurrentDrivetrain.Companion.getCurrentDrivetrain().getPedroPathingPARAMS().getXMovement();
    private static final double yMovement = CurrentDrivetrain.Companion.getCurrentDrivetrain().getPedroPathingPARAMS().getYMovement();
    private static final double[] convertToPolar = Point.cartesianToPolar(xMovement, -yMovement);
    public static Vector frontLeftVector = MathFunctions.normalizeVector(new Vector(convertToPolar[0], convertToPolar[1]));
}
