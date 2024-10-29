package org.firstinspires.ftc.teamcode.subsystems.gameSpecific

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.customHardware.servos.SynchronizedServo
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.initServo
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.setPose
import org.firstinspires.ftc.teamcode.utilClass.ServoFunc

class ScoringSubsystem(ahwMap: HardwareMap) {
    enum class ClawState {
        OPEN,
        CLOSE,
        IDLE,
    }

    private var claw: Servo
    private var clawState: ClawState = ClawState.IDLE


    enum class PitchState {
        HIGH,
        LOW,
        IDLE,
    }

    private var pitchServo: SynchronizedServo
    private var pitchState: PitchState = PitchState.IDLE

    enum class RotateState {
        LEFT,
        RIGHT,
        IDLE,
    }

    private var rotateServo: Servo
    private var rotateState: RotateState = RotateState.IDLE

    init {
        claw = initServo(ahwMap, "claw")
        pitchServo = SynchronizedServo(ahwMap, "pitchServo")
        rotateServo = initServo(ahwMap, "rotateServo")
    }

    fun update() {
        updateServos()
    }

    fun openClaw() {
        clawState = ClawState.OPEN
    }

    fun closeClaw() {
        clawState = ClawState.CLOSE
    }

    fun setPitchHigh() {
        pitchState = PitchState.HIGH
    }

    fun setPitchLow() {
        pitchState = PitchState.LOW
    }

    fun setRotateLeft() {
        rotateState = RotateState.LEFT
    }

    fun setRotateRight() {
        rotateState = RotateState.RIGHT
    }

    private fun updateServos() {
        when (clawState) {
            ClawState.OPEN -> {
                ServoFunc.openClaw(claw)
                clawState = ClawState.IDLE
            }

            ClawState.CLOSE -> {
                ServoFunc.closeClaw(claw)
                clawState = ClawState.IDLE
            }

            ClawState.IDLE -> {}
        }

        when (pitchState) {
            PitchState.HIGH -> {
                pitchServo.setPose(170.0)
                pitchState = PitchState.IDLE
            }

            PitchState.LOW -> {
                pitchServo.setPose(0.0)
                pitchState = PitchState.IDLE
            }

            PitchState.IDLE -> {}
        }

        when (rotateState) {
            RotateState.LEFT -> {
                rotateServo.setPose(170.0)
                rotateState = RotateState.IDLE
            }

            RotateState.RIGHT -> {
                rotateServo.setPose(0.0)
                rotateState = RotateState.IDLE
            }

            RotateState.IDLE -> {}
        }
    }
}