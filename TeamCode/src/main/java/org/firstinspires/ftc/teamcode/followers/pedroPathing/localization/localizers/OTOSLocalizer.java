package org.firstinspires.ftc.teamcode.followers.pedroPathing.localization.localizers;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.followers.pedroPathing.localization.Localizer;
import org.firstinspires.ftc.teamcode.followers.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.storage.CurrentDrivetrain;

/**
 * This is the OTOSLocalizer class. This class extends the Localizer superclass and is a
 * localizer that uses the SparkFun OTOS. The diagram below, which is modified from
 * Road Runner, shows a typical set up.
 * <p>
 * The view is from the top of the robot looking downwards.
 * <p>
 * left on robot is the y positive direction
 * <p>
 * forward on robot is the x positive direction
 * <p>
 * /--------------\
 * |     ____     |
 * |     ----     |
 * | ||        || |
 * | ||        || |  ----> left (y positive)
 * |              |
 * |              |
 * \--------------/
 * |
 * |
 * V
 * forward (x positive)
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 7/20/2024
 */
public class OTOSLocalizer extends Localizer {
    private final HardwareMap hardwareMap;
    private final SparkFunOTOS otos;
    private Pose startPose;
    private double previousHeading;
    private double totalHeading;

    /**
     * This creates a new OTOSLocalizer from a HardwareMap, with a starting Pose at (0,0)
     * facing 0 heading.
     *
     * @param map the HardwareMap
     */
    public OTOSLocalizer(HardwareMap map) {
        this(map, new Pose());
    }

    /**
     * This creates a new OTOSLocalizer from a HardwareMap and a Pose, with the Pose
     * specifying the starting pose of the localizer.
     *
     * @param map          the HardwareMap
     * @param setStartPose the Pose to start from
     */
    public OTOSLocalizer(HardwareMap map, Pose setStartPose) {
        hardwareMap = map;

        // TODO: replace this with your OTOS port
        otos = hardwareMap.get(SparkFunOTOS.class, CurrentDrivetrain.Companion.getCurrentDrivetrain().getSparkFunOTOSParams().getName());

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);

        // TODO: replace this with your OTOS offset from the center of the robot
        // For the OTOS, left/right is the y axis and forward/backward is the x axis, with left being
        // positive y and forward being positive x. PI/2 radians is facing forward, and clockwise
        // rotation is negative rotation.
        otos.setOffset(CurrentDrivetrain.Companion.getCurrentDrivetrain().getSparkFunOTOSParams().getOffset());

        // TODO: replace these with your tuned multipliers
        otos.setLinearScalar(CurrentDrivetrain.Companion.getCurrentDrivetrain().getSparkFunOTOSParams().getLinearScalar());
        otos.setAngularScalar(CurrentDrivetrain.Companion.getCurrentDrivetrain().getSparkFunOTOSParams().getAngularScalar());

        otos.calibrateImu();
        otos.resetTracking();

        setStartPose(setStartPose);
        totalHeading = 0;
        previousHeading = startPose.getHeading();

        resetOTOS();
    }

    /**
     * This returns the current pose estimate.
     *
     * @return returns the current pose estimate as a Pose
     */
    @Override
    public Pose getPose() {
        SparkFunOTOS.Pose2D pose = otos.getPosition();
        return MathFunctions.addPoses(startPose, new Pose(pose.x, pose.y, pose.h));
    }

    /**
     * This sets the current pose estimate. Changing this should just change the robot's current
     * pose estimate, not anything to do with the start pose.
     *
     * @param setPose the new current pose estimate
     */
    @Override
    public void setPose(Pose setPose) {
        resetOTOS();
        Pose setOTOSPose = MathFunctions.subtractPoses(setPose, startPose);
        otos.setPosition(new SparkFunOTOS.Pose2D(setOTOSPose.getX(), setOTOSPose.getY(), setOTOSPose.getHeading()));
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Pose
     */
    @Override
    public Pose getVelocity() {
        SparkFunOTOS.Pose2D OTOSVelocity = otos.getVelocity();
        return new Pose(OTOSVelocity.x, OTOSVelocity.y, OTOSVelocity.h);
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Vector
     */
    @Override
    public Vector getVelocityVector() {
        return getVelocity().getVector();
    }

    /**
     * This sets the start pose. Changing the start pose should move the robot as if all its
     * previous movements were displacing it from its new start pose.
     *
     * @param setStart the new start pose
     */
    @Override
    public void setStartPose(Pose setStart) {
        startPose = setStart;
    }

    /**
     * This updates the total heading of the robot. The OTOS handles all other updates itself.
     */
    @Override
    public void update() {
        totalHeading += MathFunctions.getSmallestAngleDifference(otos.getPosition().h, previousHeading);
        previousHeading = otos.getPosition().h;
    }

    /**
     * This resets the OTOS.
     */
    public void resetOTOS() {
        otos.resetTracking();
    }

    /**
     * This returns how far the robot has turned in radians, in a number not clamped between 0 and
     * 2 * pi radians. This is used for some tuning things and nothing actually within the following.
     *
     * @return returns how far the robot has turned in total, in radians.
     */
    public double getTotalHeading() {
        return totalHeading;
    }

    /**
     * This returns the multiplier applied to forward movement measurement to convert from OTOS
     * ticks to inches. For the OTOS, this value is the same as the lateral multiplier.
     * This is found empirically through a tuner.
     *
     * @return returns the forward ticks to inches multiplier
     */
    public double getForwardMultiplier() {
        return otos.getLinearScalar();
    }

    /**
     * This returns the multiplier applied to lateral/strafe movement measurement to convert from
     * OTOS ticks to inches. For the OTOS, this value is the same as the forward multiplier.
     * This is found empirically through a tuner.
     *
     * @return returns the lateral/strafe ticks to inches multiplier
     */
    public double getLateralMultiplier() {
        return otos.getLinearScalar();
    }

    /**
     * This returns the multiplier applied to turning movement measurement to convert from OTOS ticks
     * to radians. This is found empirically through a tuner.
     *
     * @return returns the turning ticks to radians multiplier
     */
    public double getTurningMultiplier() {
        return otos.getAngularScalar();
    }
}