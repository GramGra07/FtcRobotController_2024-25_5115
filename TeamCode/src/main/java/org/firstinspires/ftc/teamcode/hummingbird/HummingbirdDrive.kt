package org.firstinspires.ftc.teamcode.hummingbird

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower
import com.acmerobotics.roadrunner.followers.TrajectoryFollower
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.opModes.rr.drive.DriveConstants
import org.firstinspires.ftc.teamcode.opModes.rr.drive.TwoWheelTrackingLocalizer
import org.firstinspires.ftc.teamcode.opModes.rr.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.opModes.rr.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.opModes.rr.trajectorysequence.TrajectorySequenceRunner
import org.firstinspires.ftc.teamcode.opModes.rr.util.LynxModuleUtil
import org.intellij.lang.annotations.Flow
import java.util.Arrays

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
class HummingbirdDrive(hardwareMap: HardwareMap) : MecanumDrive(
    DriveConstants.kV,
    DriveConstants.kA,
    DriveConstants.kStatic,
    DriveConstants.TRACK_WIDTH,
    DriveConstants.TRACK_WIDTH,
    LATERAL_MULTIPLIER
) {
    private val leftFront: DcMotorEx
    private val leftRear: DcMotorEx
    private val rightRear: DcMotorEx
    private val rightFront: DcMotorEx
    private val motors: List<DcMotorEx>
    private val imu: IMU
    private val batteryVoltageSensor: VoltageSensor
    private val lastEncPositions: MutableList<Int> = ArrayList()
    private val lastEncVels: MutableList<Int> = ArrayList()

    init {
//        follower = HolonomicPIDVAFollower(
//            TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
//            Flower(0.5, 0.5, Math.toRadians(5.0)), 0.5
//        )
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap)
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next()
        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(IMU::class.java, "imu")
        val parameters = IMU.Parameters(
            RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR
            )
        )
        imu.initialize(parameters)
        leftFront = hardwareMap.get(DcMotorEx::class.java, "motorFrontLeft")
        leftRear = hardwareMap.get(DcMotorEx::class.java, "motorBackLeft")
        rightRear = hardwareMap.get(DcMotorEx::class.java, "motorBackRight")
        rightFront = hardwareMap.get(DcMotorEx::class.java, "motorFrontRight")
        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront)
        for (motor in motors) {
            val motorConfigurationType = motor.motorType.clone()
            motorConfigurationType.achieveableMaxRPMFraction = 1.0
            motor.motorType = motorConfigurationType
        }
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        leftRear.direction = DcMotorSimple.Direction.REVERSE
        val lastTrackingEncPositions: List<Int> = ArrayList()
        val lastTrackingEncVels: List<Int> = ArrayList()
    }
    fun update() {
        updatePoseEstimate()
        val signal = trajectorySequenceRunner.update(poseEstimate, poseVelocity)
        signal?.let { setDriveSignal(it) }
    }

    //    public void breakFollowing() {
    //        trajectorySequenceRunnerCancelable.breakFollowing();
    //    }
    fun waitForIdle() {
        while (!Thread.currentThread().isInterrupted && isBusy) update()
    }

    val isBusy: Boolean
        get() = trajectorySequenceRunner.isBusy

    fun setMode(runMode: DcMotor.RunMode?) {
        for (motor in motors) {
            motor.mode = runMode
        }
    }

    fun setZeroPowerBehavior(zeroPowerBehavior: DcMotor.ZeroPowerBehavior?) {
        for (motor in motors) {
            motor.zeroPowerBehavior = zeroPowerBehavior
        }
    }

    fun setPIDFCoefficients(runMode: DcMotor.RunMode?, coefficients: PIDFCoefficients) {
        val compensatedCoefficients = PIDFCoefficients(
            coefficients.p, coefficients.i, coefficients.d,
            coefficients.f * 12 / batteryVoltageSensor.voltage
        )
        for (motor in motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients)
        }
    }

    fun setWeightedDrivePower(drivePower: Flower) {
        var vel = drivePower
        if ((Math.abs(drivePower.x) + Math.abs(drivePower.y)
                    + Math.abs(drivePower.heading)) > 1
        ) {
            // re-normalize the powers according to the weights
            val denom =
                VX_WEIGHT * Math.abs(drivePower.x) + VY_WEIGHT * Math.abs(drivePower.y) + OMEGA_WEIGHT * Math.abs(
                    drivePower.heading
                )
            vel = Flower(
                VX_WEIGHT * drivePower.x,
                VY_WEIGHT * drivePower.y,
                OMEGA_WEIGHT * drivePower.heading
            ).div(denom)
        }
        setMotorPowers(vel)
    }

    override fun getWheelPositions(): List<Double> {
        lastEncPositions.clear()
        val wheelPositions: MutableList<Double> = ArrayList()
        for (motor in motors) {
            val position = motor.currentPosition
            lastEncPositions.add(position)
            wheelPositions.add(DriveConstants.encoderTicksToInches(position.toDouble()))
        }
        return wheelPositions
    }

    override fun getWheelVelocities(): List<Double>? {
        lastEncVels.clear()
        val wheelVelocities: MutableList<Double> = ArrayList()
        for (motor in motors) {
            val vel = motor.velocity.toInt()
            lastEncVels.add(vel)
            wheelVelocities.add(DriveConstants.encoderTicksToInches(vel.toDouble()))
        }
        return wheelVelocities
    }

    override fun setMotorPowers(v: Double, v1: Double, v2: Double, v3: Double) {
        leftFront.power = v
        leftRear.power = v1
        rightRear.power = v2
        rightFront.power = v3
    }

    override val rawExternalHeading: Double
        get() = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)

    override fun getExternalHeadingVelocity(): Double? {
        // TODO: This must be changed to match your configuration
        //                           | Z axis
        //                           |
        //     (Motor Port Side)     |   / X axis
        //                       ____|__/____
        //          Y axis     / *   | /    /|   (IO Side)
        //          _________ /______|/    //      I2C
        //                   /___________ //     Digital
        //                  |____________|/      Analog
        //
        //                 (Servo Port Side)
        //
        // The positive x axis points toward the USB port(s)
        //
        // Adjust the axis rotation rate as necessary
        // Rotate about the z axis is the default assuming your REV Hub/Control Hub is laying
        // flat on a surface
        return imu.getRobotAngularVelocity(AngleUnit.RADIANS).xRotationRate.toDouble()
    }

    fun breakFollowing() {
        trajectorySequenceRunner.breakFollowing()
    }

    companion object {
        var TRANSLATIONAL_PID = PIDCoefficients(8.0, 0.0, 0.0)
        var HEADING_PID = PIDCoefficients(8.0, 0.0, 0.0)
        var LATERAL_MULTIPLIER = 1.19
        var VX_WEIGHT = 1.0
        var VY_WEIGHT = 1.0
        var OMEGA_WEIGHT = 1.0

        //    private final TrajectorySequenceRunnerCancelable trajectorySequenceRunnerCancelable;
        private val VEL_CONSTRAINT = getVelocityConstraint(
            DriveConstants.MAX_VEL,
            DriveConstants.MAX_ANG_VEL,
            DriveConstants.TRACK_WIDTH
        )
        private val ACCEL_CONSTRAINT = getAccelerationConstraint(DriveConstants.MAX_ACCEL)
        fun getVelocityConstraint(
            maxVel: Double,
            maxAngularVel: Double,
            trackWidth: Double
        ): TrajectoryVelocityConstraint {
            return MinVelocityConstraint(
                Arrays.asList(
                    AngularVelocityConstraint(maxAngularVel),
                    MecanumVelocityConstraint(maxVel, trackWidth)
                )
            )
        }

        fun getAccelerationConstraint(maxAccel: Double): TrajectoryAccelerationConstraint {
            return ProfileAccelerationConstraint(maxAccel)
        }
    }
}
