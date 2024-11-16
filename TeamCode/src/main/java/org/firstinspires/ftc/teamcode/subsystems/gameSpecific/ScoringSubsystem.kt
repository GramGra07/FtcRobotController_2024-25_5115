package org.firstinspires.ftc.teamcode.subsystems.gameSpecific

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.customHardware.servos.AxonServo
import org.firstinspires.ftc.teamcode.customHardware.servos.SynchronizedServo
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.initServo
import org.firstinspires.ftc.teamcode.utilClass.CameraLock
import org.firstinspires.ftc.teamcode.utilClass.ServoFunc
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.ServoUtil
import kotlin.math.abs

class ScoringSubsystem(ahwMap: HardwareMap, private val armSubsystem: ArmSubsystem) {
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
        AUTO,
    }

    private var pitchServo: SynchronizedServo
    private var pitchState: PitchState = PitchState.IDLE

    enum class RotateState {
        LEFT,
        CENTER,
        IDLE,
        AUTO,
    }

    private var rotateServo: AxonServo
    private var rotateState: RotateState = RotateState.IDLE

    init {
        claw = initServo(ahwMap, "claw")
        pitchServo = SynchronizedServo(ahwMap, "pitchServo", true)
        rotateServo = AxonServo(ahwMap, "rotateServo")
    }

    fun update(lock: CameraLock?) {
        updateServos(lock)
    }

    fun setPitchAuto() {
        pitchState = PitchState.AUTO
    }

    fun setRotateAuto() {
        rotateState = RotateState.AUTO
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

    fun setPitchIdle() {
        pitchState = PitchState.IDLE
    }

    fun setRotateLeft() {
        rotateState = RotateState.LEFT
    }

    fun setRotateCenter() {
        rotateState = RotateState.CENTER
    }

    fun setRotateIdle() {
        rotateState = RotateState.IDLE
    }

    fun telemetry(telemetry: Telemetry) {
        telemetry.addData("Scoring Subsystem", "")
        rotateServo.telemetry(telemetry)
    }

    private fun updateServos(lock: CameraLock?) {
        when (clawState) {
            ClawState.OPEN -> {
                ServoFunc.openClaw(claw)
                clawState = ClawState.IDLE
                rotateState = RotateState.IDLE
                pitchState = PitchState.IDLE
            }

            ClawState.CLOSE -> {
                ServoFunc.closeClaw(claw)
                clawState = ClawState.IDLE
                rotateState = RotateState.IDLE
                pitchState = PitchState.IDLE
            }

            ClawState.IDLE -> {}
        }

        when (pitchState) {
            PitchState.HIGH -> {
                pitchServo.setPose(-60.0)
                rotateState = RotateState.IDLE
                pitchState = PitchState.IDLE
            }

            PitchState.LOW -> {
                pitchServo.setPose(80.0)
                rotateState = RotateState.IDLE
                pitchState = PitchState.IDLE
            }

            PitchState.MED -> {
                pitchServo.setPose(0.0)
                rotateState = RotateState.IDLE
                pitchState = PitchState.IDLE
            }

            PitchState.IDLE -> {
            }

            PitchState.AUTO -> {
                autoLevel()
            }
        }


        when (rotateState) {
            RotateState.LEFT -> {
                rotateServo.setPosition(ServoUtil.rotateLeft)
                rotateState = RotateState.IDLE
            }

            RotateState.CENTER -> {
                rotateServo.setPosition(ServoUtil.rotateCenter)
                rotateState = RotateState.IDLE
            }

            RotateState.IDLE -> {
            }

            RotateState.AUTO -> {
                lockRotate(lock!!)
            }
        }
    }

    fun setup() {
        setPitchHigh()
        closeClaw()
        setRotateCenter()
        update(null)
    }

    private fun lockRotate(lock: CameraLock) {
        val angle = lock.angle
        var correctedAngle = if (angle / abs(angle) == -1.0) {
            90 + angle
        } else {
            angle
        }
        rotateServo.setPosition((correctedAngle / 5) * 5)
    }

    private fun autoLevel() {
        val angle = armSubsystem.ticksPerDegree * armSubsystem.pitchEncoder.getAverage()
        val coAng = Range.clip(angle + 80, -60.0, 80.0)
        pitchServo.setPose(coAng)
    }
}