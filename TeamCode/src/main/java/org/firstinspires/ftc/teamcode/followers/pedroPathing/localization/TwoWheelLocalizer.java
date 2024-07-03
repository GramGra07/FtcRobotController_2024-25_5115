package org.firstinspires.ftc.teamcode.followers.pedroPathing.localization;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig;
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.followers.pedroPathing.util.NanoTimer;
import org.firstinspires.ftc.teamcode.storage.CurrentDrivetrain;

/**
 * This is the TwoWheelLocalizer class. This class extends the Localizer superclass and is a
 * localizer that uses the two wheel odometry with IMU set up. The diagram below, which is taken from
 * Road Runner, shows a typical set up.
 * <p>
 * The view is from the bottom of the robot looking upwards.
 * <p>
 * left on robot is y pos
 * <p>
 * front on robot is x pos
 * <p>
 * /--------------\
 * |     ____     |
 * |     ----     |
 * | ||           |
 * | ||           |   left (y pos)
 * |              |
 * |              |
 * \--------------/
 * front (x pos)
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 4/2/2024
 */
@Config
public class TwoWheelLocalizer extends Localizer {
    public static double FORWARD_TICKS_TO_INCHES = CurrentDrivetrain.Companion.getCurrentDrivetrain().getPedroPathingPARAMS().getForwardTicksToInches();
    public static double STRAFE_TICKS_TO_INCHES = CurrentDrivetrain.Companion.getCurrentDrivetrain().getPedroPathingPARAMS().getLateralTicksToInches();
    private final HardwareMap hardwareMap;
    private final IMU imu;
    private final NanoTimer timer;
    private final Encoder forwardEncoder;
    private final Encoder strafeEncoder;
    private final Pose forwardEncoderPose;
    private final Pose strafeEncoderPose;
    private Pose startPose;
    private Pose displacementPose;
    private Pose currentVelocity;
    private Matrix prevRotationMatrix;
    private long deltaTimeNano;
    private double previousIMUOrientation;
    private double deltaRadians;
    private double totalHeading;

    /**
     * This creates a new TwoWheelLocalizer from a HardwareMap, with a starting Pose at (0,0)
     * facing 0 heading.
     *
     * @param map the HardwareMap
     */
    public TwoWheelLocalizer(HardwareMap map) {
        this(map, new Pose());
    }

    /**
     * This creates a new TwoWheelLocalizer from a HardwareMap and a Pose, with the Pose
     * specifying the starting pose of the localizer.
     *
     * @param map          the HardwareMap
     * @param setStartPose the Pose to start from
     */
    public TwoWheelLocalizer(HardwareMap map, Pose setStartPose) {
        // TODO: replace these with your encoder positions
        forwardEncoderPose = CurrentDrivetrain.Companion.getCurrentDrivetrain().getPedroPathingPARAMS().getForwardEncoderPose();
        strafeEncoderPose = CurrentDrivetrain.Companion.getCurrentDrivetrain().getPedroPathingPARAMS().getLateralEncoderPose();

        hardwareMap = map;

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(CurrentDrivetrain.Companion.getCurrentDrivetrain().getDriveConfigPARAMS().getLogoFacingDirection(), CurrentDrivetrain.Companion.getCurrentDrivetrain().getDriveConfigPARAMS().getUsbFacingDirection())));

        // TODO: replace these with your encoder ports
        forwardEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, CurrentDrivetrain.Companion.getCurrentDrivetrain().getDriveConfigPARAMS().getParEncoder()));
        strafeEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, CurrentDrivetrain.Companion.getCurrentDrivetrain().getDriveConfigPARAMS().getPerEncoder()));

        forwardEncoder.setDirection(Encoder.REVERSE);
        if (HardwareConfig.Companion.isTesterDrivetrain())
            strafeEncoder.setDirection(Encoder.REVERSE);

        setStartPose(setStartPose);
        timer = new NanoTimer();
        deltaTimeNano = 1;
        displacementPose = new Pose();
        currentVelocity = new Pose();

        previousIMUOrientation = MathFunctions.normalizeAngle(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        deltaRadians = 0;
    }

    /**
     * This returns the current pose estimate.
     *
     * @return returns the current pose estimate as a Pose
     */
    @Override
    public Pose getPose() {
        return MathFunctions.addPoses(startPose, displacementPose);
    }

    /**
     * This sets the current pose estimate. Changing this should just change the robot's current
     * pose estimate, not anything to do with the start pose.
     *
     * @param setPose the new current pose estimate
     */
    @Override
    public void setPose(Pose setPose) {
        displacementPose = MathFunctions.subtractPoses(setPose, startPose);
        resetEncoders();
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Pose
     */
    @Override
    public Pose getVelocity() {
        return currentVelocity.copy();
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Vector
     */
    @Override
    public Vector getVelocityVector() {
        return currentVelocity.getVector();
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
     * This sets the Matrix that contains the previous pose's heading rotation.
     *
     * @param heading the rotation of the Matrix
     */
    public void setPrevRotationMatrix(double heading) {
        prevRotationMatrix = new Matrix(3, 3);
        prevRotationMatrix.set(0, 0, Math.cos(heading));
        prevRotationMatrix.set(0, 1, -Math.sin(heading));
        prevRotationMatrix.set(1, 0, Math.sin(heading));
        prevRotationMatrix.set(1, 1, Math.cos(heading));
        prevRotationMatrix.set(2, 2, 1.0);
    }

    /**
     * This updates the elapsed time timer that keeps track of time between updates, as well as the
     * change position of the Encoders and the IMU readings. Then, the robot's global change in
     * position is calculated using the pose exponential method.
     */
    @Override
    public void update() {
        deltaTimeNano = timer.getElapsedTime();
        timer.resetTimer();

        updateEncoders();
        Matrix robotDeltas = getRobotDeltas();
        Matrix globalDeltas;
        setPrevRotationMatrix(getPose().getHeading());

        Matrix transformation = new Matrix(3, 3);
        if (Math.abs(robotDeltas.get(2, 0)) < 0.001) {
            transformation.set(0, 0, 1.0 - (Math.pow(robotDeltas.get(2, 0), 2) / 6.0));
            transformation.set(0, 1, -robotDeltas.get(2, 0) / 2.0);
            transformation.set(1, 0, robotDeltas.get(2, 0) / 2.0);
            transformation.set(1, 1, 1.0 - (Math.pow(robotDeltas.get(2, 0), 2) / 6.0));
            transformation.set(2, 2, 1.0);
        } else {
            transformation.set(0, 0, Math.sin(robotDeltas.get(2, 0)) / robotDeltas.get(2, 0));
            transformation.set(0, 1, (Math.cos(robotDeltas.get(2, 0)) - 1.0) / robotDeltas.get(2, 0));
            transformation.set(1, 0, (1.0 - Math.cos(robotDeltas.get(2, 0))) / robotDeltas.get(2, 0));
            transformation.set(1, 1, Math.sin(robotDeltas.get(2, 0)) / robotDeltas.get(2, 0));
            transformation.set(2, 2, 1.0);
        }

        globalDeltas = Matrix.multiply(Matrix.multiply(prevRotationMatrix, transformation), robotDeltas);

        displacementPose.add(new Pose(globalDeltas.get(0, 0), globalDeltas.get(1, 0), globalDeltas.get(2, 0)));
        currentVelocity = new Pose(globalDeltas.get(0, 0) / (deltaTimeNano * Math.pow(10.0, 9)), globalDeltas.get(1, 0) / (deltaTimeNano * Math.pow(10.0, 9)), globalDeltas.get(2, 0) / (deltaTimeNano * Math.pow(10.0, 9)));

        totalHeading += globalDeltas.get(2, 0);
    }

    /**
     * This updates the Encoders as well as the IMU.
     */
    public void updateEncoders() {
        forwardEncoder.update();
        strafeEncoder.update();

        double currentIMUOrientation = MathFunctions.normalizeAngle(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        deltaRadians = MathFunctions.getTurnDirection(previousIMUOrientation, currentIMUOrientation) * MathFunctions.getSmallestAngleDifference(currentIMUOrientation, previousIMUOrientation);
        previousIMUOrientation = currentIMUOrientation;
    }

    /**
     * This resets the Encoders.
     */
    public void resetEncoders() {
        forwardEncoder.reset();
        strafeEncoder.reset();
    }

    /**
     * This calculates the change in position from the perspective of the robot using information
     * from the Encoders and IMU.
     *
     * @return returns a Matrix containing the robot relative movement.
     */
    public Matrix getRobotDeltas() {
        Matrix returnMatrix = new Matrix(3, 1);
        // x/forward movement
        returnMatrix.set(0, 0, FORWARD_TICKS_TO_INCHES * (forwardEncoder.getDeltaPosition() - forwardEncoderPose.getY() * deltaRadians));
        //y/strafe movement
        returnMatrix.set(1, 0, STRAFE_TICKS_TO_INCHES * (strafeEncoder.getDeltaPosition() - strafeEncoderPose.getX() * deltaRadians));
        // theta/turning
        returnMatrix.set(2, 0, deltaRadians);
        return returnMatrix;
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
     * This returns the multiplier applied to forward movement measurement to convert from encoder
     * ticks to inches. This is found empirically through a tuner.
     *
     * @return returns the forward ticks to inches multiplier
     */
    public double getForwardMultiplier() {
        return FORWARD_TICKS_TO_INCHES;
    }

    /**
     * This returns the multiplier applied to lateral/strafe movement measurement to convert from
     * encoder ticks to inches. This is found empirically through a tuner.
     *
     * @return returns the lateral/strafe ticks to inches multiplier
     */
    public double getLateralMultiplier() {
        return STRAFE_TICKS_TO_INCHES;
    }

    /**
     * This returns the multiplier applied to turning movement measurement to convert from encoder
     * ticks to radians. This is found empirically through a tuner.
     *
     * @return returns the turning ticks to radians multiplier
     */
    public double getTurningMultiplier() {
        return 1;
    }
}
