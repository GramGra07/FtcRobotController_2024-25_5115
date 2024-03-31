package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.controller.PIDFController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.UtilClass.varConfigurations.PIDVals
import org.firstinspires.ftc.teamcode.extensions.MotorExtensions.initMotor
import kotlin.math.abs

class ExtendoSubsystem(ahwMap: HardwareMap) {
    enum class RotateState {
        CONTROLLED_PID,
        CONTROL,
        STOPPED,
        IDLE
    }

    enum class ExtendState {
        CONTROLLED_PID,
        CONTROL,
        STOPPED,
        IDLE
    }

    private var rotateState: RotateState = RotateState.STOPPED
    private var extendState: ExtendState = ExtendState.STOPPED
    private var motorExtension: DcMotor
    private var motorRotation: DcMotor
    private var ePower = 0.0
    private var rPower = 0.0
    var usePIDF = true
    private var extensionPIDF = PIDFController(0.0, 0.0, 0.0, 0.0)
    private var rotationPIDF = PIDFController(0.0, 0.0, 0.0, 0.0)

    @JvmField
    var flipperMin = -0.7

    @JvmField
    var flipperMax = 0.7

    @JvmField
    var slideMax = 1.0

    @JvmField
    var slideMin = -1.0

    @JvmField
    var autoExtension = 1100

    @JvmField
    var autoRotation = 800

    @JvmField
    var maxExtensionTicks = 3280

    @JvmField
    var minExtensionTicks = 0

    @JvmField
    var maxRotationTicks = 1424

    @JvmField
    var maxPotent = 65

    @JvmField
    var minRotationTicks = 0

    init {
        motorExtension = initMotor(ahwMap, "slideMotor", DcMotor.RunMode.RUN_USING_ENCODER)
        motorRotation = initMotor(
            ahwMap,
            "flipperMotor",
            DcMotor.RunMode.RUN_USING_ENCODER,
            DcMotorSimple.Direction.REVERSE
        )

        updatePID()
    }


    fun setPowerR(input: Double) {
        updatePID()
        when (rotateState) {
            RotateState.CONTROLLED_PID -> {
                rPower = calculatePID(rotationPIDF, motorRotation.currentPosition.toDouble(), input)
            }

            RotateState.CONTROL -> {
                rPower = Range.clip(
                    input,
                    flipperMin,
                    flipperMax
                )
            }

            RotateState.STOPPED -> {
                rPower = 0.0
            }

            RotateState.IDLE -> {}

        }
    }

    fun setPowerE(input: Double) {
        updatePID()
        when (extendState) {
            ExtendState.CONTROLLED_PID -> {
                ePower =
                    calculatePID(extensionPIDF, motorExtension.currentPosition.toDouble(), input)
            }

            ExtendState.CONTROL -> {
                ePower = Range.clip(
                    input,
                    slideMin,
                    slideMax
                )
            }

            ExtendState.STOPPED -> {
                ePower = 0.0
            }

            ExtendState.IDLE -> {}
        }
    }

    fun stopR() {
        updatePID()
        rPower = 0.0
        rotateState = RotateState.STOPPED
    }

    fun stopE() {
        updatePID()
        ePower = 0.0
        extendState = ExtendState.STOPPED
    }

    fun autoExtend(position: Int, driveSubsystem: DriveSubsystem) {
        val drive = driveSubsystem.drive
        motorExtension.mode = DcMotor.RunMode.RUN_USING_ENCODER
        motorExtension.targetPosition = (motorExtension.currentPosition + position)
        drive.updatePoseEstimate()
        motorExtension.mode = DcMotor.RunMode.RUN_TO_POSITION
        motorExtension.power = abs(1).toDouble()
        while (motorExtension.isBusy) {
            drive.updatePoseEstimate()
        }
        motorExtension.power = 0.0
    }

    fun autoRotate(position: Int, driveSubsystem: DriveSubsystem) {
        val drive = driveSubsystem.drive
        motorRotation.mode = DcMotor.RunMode.RUN_USING_ENCODER
        motorRotation.targetPosition = (motorRotation.currentPosition + position)
        drive.updatePoseEstimate()
        motorRotation.mode = DcMotor.RunMode.RUN_TO_POSITION
        motorRotation.power = abs(1).toDouble()
        while (motorRotation.isBusy) {
            drive.updatePoseEstimate()
        }
        motorRotation.power = 0.0
    }

    private fun power() {
        when (rotateState) {
            RotateState.CONTROL,
            RotateState.CONTROLLED_PID -> {
                motorExtension.power = ePower
            }

            RotateState.STOPPED -> {
                motorExtension.power = 0.0
            }

            RotateState.IDLE -> {}
        }
        when (extendState) {
            ExtendState.CONTROL,
            ExtendState.CONTROLLED_PID -> {
                motorRotation.power = rPower
            }

            ExtendState.STOPPED -> {
                motorRotation.power = 0.0
            }

            ExtendState.IDLE -> {}
        }
    }

    fun update() {
        updatePID()
        power()
    }

    private fun updatePID() {
        if (usePIDF) {
            extendState = ExtendState.CONTROLLED_PID
            rotateState = RotateState.CONTROLLED_PID
        } else {
            extendState = ExtendState.CONTROL
            rotateState = RotateState.CONTROL
        }
        rotationPIDF.setPIDF(
            PIDVals.rotationPIDFCo.p,
            PIDVals.rotationPIDFCo.i,
            PIDVals.rotationPIDFCo.d,
            PIDVals.rotationPIDFCo.f
        )
        extensionPIDF.setPIDF(
            PIDVals.extensionPIDFCo.p,
            PIDVals.extensionPIDFCo.i,
            PIDVals.extensionPIDFCo.d,
            PIDVals.extensionPIDFCo.f
        )
    }

    private fun calculatePID(controller: PIDFController, current: Double, target: Double): Double {
        return Range.clip(
            controller.calculate(
                current,
                target
            ), -1.0, 1.0
        )
    }

    fun idleE() {
        extendState = ExtendState.IDLE
    }

    fun idleR() {
        rotateState = RotateState.IDLE
    }

    fun telemetry(telemetry: Telemetry) {
        telemetry.addData("Extension Position", motorExtension.currentPosition)
        telemetry.addData("Rotation Position", motorRotation.currentPosition)
    }
}