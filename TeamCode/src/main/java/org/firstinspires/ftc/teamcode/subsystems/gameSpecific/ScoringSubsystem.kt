package org.firstinspires.ftc.teamcode.subsystems.gameSpecific

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.customHardware.servos.AxonServo
import org.firstinspires.ftc.teamcode.customHardware.servos.SynchronizedServo
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.initServo
import org.firstinspires.ftc.teamcode.utilClass.CameraLock
import org.firstinspires.ftc.teamcode.utilClass.ServoFunc
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.ServoUtil
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.VarConfig.autoRotateClaw
import kotlin.math.abs

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
        MED,
        IDLE,
    }

    private var pitchServo: SynchronizedServo
    private var pitchState: PitchState = PitchState.IDLE

    enum class RotateState {
        LEFT,
        CENTER,
        IDLE,
    }

    enum class RotateMode {
        AUTO,
        SEMI_AUTO,
        MANUAL,
    }

    private var rotateServo: AxonServo
    private var rotateState: RotateState = RotateState.IDLE
    private var rotateMode: RotateMode = RotateMode.MANUAL

    init {
        claw = initServo(ahwMap, "claw")
        pitchServo = SynchronizedServo(ahwMap, "pitchServo", true)
        rotateServo = AxonServo(ahwMap, "rotateServo")
    }

    fun update(lock: CameraLock) {
        updateServos(lock)
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

    fun setPitchMed() {
        pitchState = PitchState.MED
    }

    fun setRotateLeft() {
        rotateState = RotateState.LEFT
    }

    fun setRotateCenter() {
        rotateState = RotateState.CENTER
    }

    fun telemetry(telemetry: Telemetry) {
        telemetry.addData("Scoring Subsystem", "")
        rotateServo.telemetry(telemetry)
    }

    private fun updateServos(lock: CameraLock) {
        when (clawState) {
            ClawState.OPEN -> {
                ServoFunc.openClaw(claw)
                clawState = ClawState.IDLE
                rotateMode = RotateMode.SEMI_AUTO
            }

            ClawState.CLOSE -> {
                ServoFunc.closeClaw(claw)
                clawState = ClawState.IDLE
                rotateMode = RotateMode.MANUAL
            }

            ClawState.IDLE -> {}
        }

        when (pitchState) {
            PitchState.HIGH -> {
                pitchServo.setPose(-90.0)
                pitchState = PitchState.IDLE
                rotateMode = RotateMode.MANUAL
            }

            PitchState.LOW -> {
                pitchServo.setPose(90.0)
                pitchState = PitchState.IDLE
                rotateMode = RotateMode.SEMI_AUTO
            }

            PitchState.MED -> {
                pitchServo.setPose(90.0)
                pitchState = PitchState.IDLE
                rotateMode = RotateMode.MANUAL
            }

            PitchState.IDLE -> {}
        }
        if (lock.size) {
            rotateMode = RotateMode.MANUAL
        }
        if (autoRotateClaw) {
            rotateMode = RotateMode.MANUAL

        }
        when (rotateMode) {
            RotateMode.AUTO -> {
                lockRotate(lock)
            }

            RotateMode.SEMI_AUTO -> {
                when (rotateState) {
                    RotateState.LEFT -> {
                        rotateServo.setPosition(ServoUtil.rotateLeft)
                    }

                    RotateState.CENTER -> {
                        rotateServo.setPosition(ServoUtil.rotateCenter)
                    }

                    RotateState.IDLE -> {
                        lockRotate(lock)
                    }
                }
            }

            RotateMode.MANUAL -> {
                rotateManual()
            }
        }


    }

    private fun rotateManual() {
        when (rotateState) {
            RotateState.LEFT -> {
                rotateServo.setPosition(ServoUtil.rotateLeft)
                rotateState = RotateState.IDLE
            }

            RotateState.CENTER -> {
                rotateServo.setPosition(ServoUtil.rotateCenter)
                rotateState = RotateState.IDLE
            }

            RotateState.IDLE -> {}
        }
    }

    fun setup() {
        pitchServo.setPose(0.0)
        rotateServo.setPosition(ServoUtil.rotateCenter)
    }

    private fun lockRotate(lock: CameraLock) {
        val angle = lock.angle
        var correctedAngle = 0.0
        correctedAngle = if (angle / abs(angle) == -1.0) {
            90 + angle
        } else {
            angle
        }
        rotateServo.setPosition(correctedAngle)
    }
}