package org.firstinspires.ftc.teamcode.subsystems.gameSpecific

import com.arcrobotics.ftclib.controller.PIDFController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.customHardware.servos.AxonServo
import org.firstinspires.ftc.teamcode.extensions.MotorExtensions.initMotor
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.calcFlipPose
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.initServo
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.PIDVals
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.ServoUtil

class ScoringSubsystem(ahwMap: HardwareMap) {
    enum class ExtensionState {
        PID,
        MANUAL,
        STOPPED,
        IDLE,
    }

    enum class RotationState {
        PID,
        MANUAL,
        STOPPED,
        IDLE,
    }


    enum class PitchState {
        HIGH, LOW,
        IDLE,
    }

    enum class TopPitchState {
        HIGH, LOW,
        IDLE,
    }

    enum class ClawState {
        OPEN,
        CLOSE,
        IDLE,
    }


    private var motorExtension: DcMotor
//    private var motorRotation: DcMotor
    private var ePower: Double = 0.0
//    private var rPower: Double = 0.0
    private var extensionState: ExtensionState = ExtensionState.IDLE
//    private var rotationState: RotationState = RotationState.IDLE
    var usePIDF = true
    private var extensionPIDF = PIDFController(0.0, 0.0, 0.0, 0.0)
//    private var rotationPIDF = PIDFController(0.0, 0.0, 0.0, 0.0)
    private var eMin = -1.0
    private var eMax = 1.0
//    private var rMin = -1.0
//    private var rMax = 1.0
//    var rotationMaxTicks = 1000
//    var rotationMinTicks = 0
    var extensionMaxTicks = 1000
    var extensionMinTicks = 0

    private var claw: Servo
    private var pitchServo: AxonServo
    private var topPitchServo: Servo

    private var clawState: ClawState = ClawState.IDLE
    private var pitchState: PitchState = PitchState.IDLE
    private var topPitchState: TopPitchState = TopPitchState.IDLE

    init {
        motorExtension = initMotor(ahwMap, "motorExtension", DcMotor.RunMode.RUN_USING_ENCODER)
//        motorRotation = initMotor(ahwMap, "motorRotation", DcMotor.RunMode.RUN_USING_ENCODER)

        claw = initServo(ahwMap, "claw")
        pitchServo = AxonServo(ahwMap, "pitchServo", 0.0)
        topPitchServo = initServo(ahwMap, "topPitchServo")
        updatePID()
    }

//    fun setPowerR(target: Double) {
//        updatePID()
//        when (rotationState) {
//            RotationState.PID -> {
//                rPower =
//                    calculatePID(rotationPIDF, motorRotation.currentPosition.toDouble(), target)
//            }
//
//            RotationState.MANUAL -> {
//                rPower = Range.clip(
//                    target,
//                    rMin,
//                    rMax
//                )
//            }
//
//            RotationState.STOPPED -> {
//                rPower = 0.0
//            }
//
//            RotationState.IDLE -> {}
//        }
//    }

    fun setPowerE(target: Double) {
        updatePID()
        when (extensionState) {
            ExtensionState.PID -> {
                ePower =
                    calculatePID(extensionPIDF, motorExtension.currentPosition.toDouble(), target)
            }

            ExtensionState.MANUAL -> {
                ePower = Range.clip(
                    target,
                    eMin,
                    eMax
                )
            }

            ExtensionState.STOPPED -> {
                ePower = 0.0
            }

            ExtensionState.IDLE -> {}
        }
    }

//    fun stopR() {
//        updatePID()
//        rPower = 0.0
//        rotationState = RotationState.STOPPED
//    }

    fun stopE() {
        updatePID()
        ePower = 0.0
        extensionState = ExtensionState.STOPPED
    }

    private fun power() {
        when (extensionState) {
            ExtensionState.MANUAL,
            ExtensionState.PID -> {
                motorExtension.power = ePower
            }

            ExtensionState.STOPPED -> {
                motorExtension.power = 0.0
                idleE()
            }

            ExtensionState.IDLE -> {}
        }

//        when (rotationState) {
//            RotationState.MANUAL,
//            RotationState.PID -> {
//                motorExtension.power = ePower
//            }
//
//            RotationState.STOPPED -> {
//                motorExtension.power = 0.0
//                idleR()
//            }
//
//            RotationState.IDLE -> {}
//        }
    }

    fun update() {
        updateServos()
        updatePID()
        power()
    }

    private fun updatePID() {
        if (usePIDF) {
            extensionState = ExtensionState.PID
//            rotationState = RotationState.PID
        } else {
            extensionState = ExtensionState.MANUAL
//            rotationState = RotationState.MANUAL
        }
        extensionPIDF.setPIDF(
            PIDVals.extensionPIDFCo.p,
            PIDVals.extensionPIDFCo.i,
            PIDVals.extensionPIDFCo.d,
            PIDVals.extensionPIDFCo.f
        )

//        rotationPIDF.setPIDF(
//            PIDVals.rotationPIDFCo.p,
//            PIDVals.rotationPIDFCo.i,
//            PIDVals.rotationPIDFCo.d,
//            PIDVals.rotationPIDFCo.f
//        )
    }

    private fun calculatePID(controller: PIDFController, current: Double, target: Double): Double {
        return Range.clip(
            controller.calculate(
                current,
                target
            ), -1.0, 1.0
        )
    }

//    fun idleR() {
//        rotationState = RotationState.IDLE
//    }

    fun idleE() {
        extensionState = ExtensionState.IDLE
    }

    fun telemetry(telemetry: Telemetry) {
        telemetry.addData("Scoring Subsystem", "")
        telemetry.addData("Extension Position", motorExtension.currentPosition)
//        telemetry.addData("Rotation Position", motorRotation.currentPosition)
        pitchServo.telemetry(telemetry)
    }

    fun openClaw() {
        clawState = ClawState.OPEN
    }

    fun closeClaw() {
        clawState = ClawState.CLOSE
    }

    fun pitchHigh() {
        pitchState = PitchState.HIGH
    }

    fun pitchLow() {
        pitchState = PitchState.LOW
    }

    fun topPitchHigh() {
        topPitchState = TopPitchState.HIGH
    }

    fun topPitchLow() {
        topPitchState = TopPitchState.LOW
    }

    private fun updateServos() {
        when (clawState) {
            ClawState.OPEN -> {
                ServoUtil.openClaw(claw)
                clawState = ClawState.IDLE
            }

            ClawState.CLOSE -> {
                ServoUtil.closeClaw(claw)
                clawState = ClawState.IDLE
            }

            ClawState.IDLE -> {}
        }
        when (pitchState) {
            PitchState.HIGH -> {
                pitchServo.calcFlipPose(70.0)
                pitchState = PitchState.IDLE
            }

            PitchState.LOW -> {
                pitchServo.calcFlipPose(0.0)
                pitchState = PitchState.IDLE
            }

            PitchState.IDLE -> {}
        }
        when (topPitchState) {
            TopPitchState.HIGH -> {
                topPitchServo.calcFlipPose(0.0)
                topPitchState = TopPitchState.IDLE
            }

            TopPitchState.LOW -> {
                topPitchServo.calcFlipPose(0.0)
                topPitchState = TopPitchState.IDLE
            }

            TopPitchState.IDLE -> {}
        }
    }
}