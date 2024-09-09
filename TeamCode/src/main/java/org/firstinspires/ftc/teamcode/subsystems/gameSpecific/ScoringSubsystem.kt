package org.firstinspires.ftc.teamcode.subsystems.gameSpecific

import com.arcrobotics.ftclib.controller.PIDFController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.customHardware.servos.AxonServo
import org.firstinspires.ftc.teamcode.extensions.MotorExtensions.initMotor
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.initServo
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.setPose
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
        FORWARD, REVERSE,
        IDLE,
    }

    enum class ClawState {
        OPEN,
        CLOSE,
        IDLE,
    }


    private var motorExtension: DcMotor
    private var extensionState: ExtensionState = ExtensionState.IDLE
    var usePIDF = false
    private var extensionPIDF = PIDFController(0.0, 0.0, 0.0, 0.0)
    private var eMin = -1.0
    private var eMax = 1.0
    var extensionMaxTicks = 1000
    var extensionMinTicks = 0

    private var claw: Servo
    private var pitchServo: Servo
    private var topPitchServo: AxonServo

    private var clawState: ClawState = ClawState.IDLE
    private var pitchState: PitchState = PitchState.IDLE
    private var topPitchState: TopPitchState = TopPitchState.IDLE

    init {
        motorExtension = initMotor(ahwMap, "motorExtension", DcMotor.RunMode.RUN_USING_ENCODER)

        claw = initServo(ahwMap, "claw")
        pitchServo = initServo(ahwMap, "pitchServo")
        topPitchServo = AxonServo(ahwMap, "topPitchServo", 0.0)
        updatePID()
    }

    fun setPowerE(target: Double) {
//        updatePID()
        when (extensionState) {
            ExtensionState.PID -> {
                motorExtension.power =
                    calculatePID(extensionPIDF, motorExtension.currentPosition.toDouble(), target)
            }

            ExtensionState.MANUAL -> {
                motorExtension.power = Range.clip(
                    target,
                    eMin,
                    eMax
                )
            }

            ExtensionState.STOPPED -> {
                motorExtension.power = 0.0
            }

            ExtensionState.IDLE -> {}
        }
    }

    fun stopE() {
        motorExtension.power = 0.0
        extensionState = ExtensionState.STOPPED
    }

    fun update() {
        updateServos()
//        updatePID()
    }

    private fun updatePID() {
        if (usePIDF) {
            extensionState = ExtensionState.PID
        } else {
            extensionState = ExtensionState.MANUAL
        }
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

    fun telemetry(telemetry: Telemetry) {
        telemetry.addData("Scoring Subsystem", "")
        telemetry.addData("Extension Position", motorExtension.currentPosition)
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

    fun topPitchForward() {
        topPitchState = TopPitchState.FORWARD
    }

    fun topPitchReverse() {
        topPitchState = TopPitchState.REVERSE
    }

    fun idleTopPitch() {
        topPitchState = TopPitchState.IDLE
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
                pitchServo.setPose(ServoUtil.pitchHigh)
                pitchState = PitchState.IDLE
            }

            PitchState.LOW -> {
                pitchServo.setPose(ServoUtil.pitchLow)
                pitchState = PitchState.IDLE
            }

            PitchState.IDLE -> {}
        }
        when (topPitchState) {
            TopPitchState.FORWARD -> {
                topPitchServo.setPosition(ServoUtil.topPitchHigh)
            }

            TopPitchState.REVERSE -> {
                topPitchServo.setPosition(ServoUtil.topPitchLow)
            }

            TopPitchState.IDLE -> {
            }
        }
    }
}