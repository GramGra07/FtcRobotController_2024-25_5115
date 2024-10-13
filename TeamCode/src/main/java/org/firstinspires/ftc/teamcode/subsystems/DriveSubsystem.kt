package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.extensions.MotorExtensions.initMotor
import org.firstinspires.ftc.teamcode.subsystems.humanInput.Drivers
import org.firstinspires.ftc.teamcode.utilClass.drivetrain.Drivetrain
import org.firstinspires.ftc.teamcode.utilClass.drivetrain.DrivetrainType
import org.firstinspires.ftc.teamcode.utilClass.objects.DriveType
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.VarConfig
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin


//@Config
class DriveSubsystem(
    ahwMap: HardwareMap,
    private var localizerSubsystem: LocalizerSubsystem,
    private var dt: Drivetrain
) {
    private var motorFrontLeft: DcMotorEx
    private var motorBackLeft: DcMotorEx
    private var motorFrontRight: DcMotorEx
    private var motorBackRight: DcMotorEx

    init {
        motorFrontLeft =
            initMotor(
                ahwMap,
                "motorFrontLeft",
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
            )
        motorBackLeft = initMotor(
            ahwMap,
            "motorBackLeft",
            DcMotor.RunMode.RUN_WITHOUT_ENCODER,
        )
        motorFrontRight =
            initMotor(
                ahwMap,
                "motorFrontRight",
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
            )
        motorBackRight =
            initMotor(
                ahwMap,
                "motorBackRight",
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
            )
        motorBackLeft.direction = DcMotorSimple.Direction.REVERSE
        motorFrontLeft.direction = DcMotorSimple.Direction.REVERSE
    }

    private var frontRightPower: Double = 0.0
    private var frontLeftPower = 0.0
    private var backRightPower = 0.0
    private var backLeftPower = 0.0
    private var setAPower = false
    var slowModeIsOn = false
    private var reverse = false
    private var isAutoInTeleop = false
    var leftStickX = 0.0
    var leftStickY = 0.0
    var rightStickX = 0.0
    fun driveByGamepads(type: DriveType, myOpMode: OpMode) {
        // Retrieve gamepad values
        leftStickX = -myOpMode.gamepad1.left_stick_x.toDouble()
        leftStickY = -myOpMode.gamepad1.left_stick_y.toDouble()
        rightStickX = -myOpMode.gamepad1.right_stick_x.toDouble()

        val slowPower = if (slowModeIsOn) VarConfig.slowMult else 1.0
//
        if (dt.type == DrivetrainType.MECANUM) {
            if (type == DriveType.FIELD_CENTRIC) {
                val controllerAngle = Math.toDegrees(atan2(leftStickY, leftStickX))
                val robotDegree = Math.toDegrees(localizerSubsystem.heading())
                val movementDegree = controllerAngle - robotDegree
                val gamepadHypot = Range.clip(hypot(leftStickX, leftStickY), 0.0, 1.0)

                // Compute x and y controls
                val yControl = cos(Math.toRadians(movementDegree)) * gamepadHypot
                val xControl = -(sin(Math.toRadians(movementDegree)) * gamepadHypot)

                // Compute powers
                val turn = rightStickX
                frontRightPower =
                    (yControl * abs(yControl) - xControl * abs(xControl) + turn) / slowPower
                backRightPower =
                    (yControl * abs(yControl) + xControl * abs(xControl) + turn) / slowPower
                frontLeftPower =
                    (yControl * abs(yControl) + xControl * abs(xControl) - turn) / slowPower
                backLeftPower =
                    (yControl * abs(yControl) - xControl * abs(xControl) - turn) / slowPower
            } else if (type == DriveType.ROBOT_CENTRIC) {
                val turn = rightStickX
                frontRightPower =
                    (leftStickY * abs(leftStickY) - leftStickX * abs(leftStickX) + turn) / slowPower
                backRightPower =
                    (leftStickY * abs(leftStickY) + leftStickX * abs(leftStickX) + turn) / slowPower
                frontLeftPower =
                    (leftStickY * abs(leftStickY) + leftStickX * abs(leftStickX) - turn) / slowPower
                backLeftPower =
                    (leftStickY * abs(leftStickY) - leftStickX * abs(leftStickX) - turn) / slowPower
            }
        } else if (dt.type == DrivetrainType.TANK) {
            frontLeftPower = Range.clip(leftStickY - rightStickX, -1.0, 1.0)
            backLeftPower = frontLeftPower
            frontRightPower = Range.clip(leftStickY + rightStickX, -1.0, 1.0)
            backRightPower = frontRightPower
        }
    }

    private fun power(
    ) {
        if (!isAutoInTeleop) {
//            var flP = 0.0
//            var frP = 0.0
//            var rrP = 0.0
//            var rlP = 0.0
//            if (avoidanceSubsystem.powers != null && type != AvoidanceSubsystem.AvoidanceTypes.OFF) {
//                val addedPowers: Map<String, Double?>? = avoidanceSubsystem.powers
//                flP = addedPowers?.getOrDefault("FL", 0.0) ?: 0.0
//                frP = addedPowers?.getOrDefault("FR", 0.0) ?: 0.0
//                rlP = addedPowers?.getOrDefault("RL", 0.0) ?: 0.0
//                rrP = addedPowers?.getOrDefault("RR", 0.0) ?: 0.0
//            }

            frontLeftPower = Range.clip(frontLeftPower, -1.0, 1.0)
            frontRightPower = Range.clip(frontRightPower, -1.0, 1.0)
            backLeftPower = Range.clip(backLeftPower, -1.0, 1.0)
            backRightPower = Range.clip(backRightPower, -1.0, 1.0)

            motorFrontLeft.power = frontLeftPower
            motorBackLeft.power = backLeftPower
            motorFrontRight.power = frontRightPower
            motorBackRight.power = backRightPower
        }
    }

    fun setArtificialPower(fwd: Double, axial: Double) {
        setAPower = true
        frontLeftPower = Range.clip(fwd + axial, -1.0, 1.0)
        backLeftPower = frontLeftPower
        frontRightPower = Range.clip(fwd - axial, -1.0, 1.0)
        backRightPower = frontRightPower
        frontLeftPower = Range.clip(frontLeftPower, -1.0, 1.0)
        frontRightPower = Range.clip(frontRightPower, -1.0, 1.0)
        backLeftPower = Range.clip(backLeftPower, -1.0, 1.0)
        backRightPower = Range.clip(backRightPower, -1.0, 1.0)
        motorFrontLeft.power = frontLeftPower
        motorBackLeft.power = backLeftPower
        motorFrontRight.power = frontRightPower
        motorBackRight.power = backRightPower
    }


    fun update(
    ) {
        power()
    }

    fun telemetry(telemetry: Telemetry, minimal: Boolean) {
        telemetry.addData("DRIVE", "")
        if (!minimal) {
            dt.telemetry(telemetry)
            if (reverse) {
                telemetry.addData("reversed", "")
            }
            when (Drivers.currentFieldCentric) {
                DriveType.FIELD_CENTRIC -> telemetry.addData("fieldCentric", "")
                DriveType.ROBOT_CENTRIC -> telemetry.addData("robotCentric", "")
            }
        }
        if (slowModeIsOn) {
            telemetry.addData("slowMode", "")
        }
    }
}