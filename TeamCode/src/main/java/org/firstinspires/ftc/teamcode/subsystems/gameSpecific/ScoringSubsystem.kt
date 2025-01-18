package org.firstinspires.ftc.teamcode.subsystems.gameSpecific

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.toBinary2
import org.firstinspires.ftc.teamcode.customHardware.camera.camUtil.CameraUtilities.initializeProcessor
import org.firstinspires.ftc.teamcode.customHardware.camera.camUtil.CameraUtilities.targetLockProcessor
import org.firstinspires.ftc.teamcode.customHardware.camera.camUtil.PROCESSORS
import org.firstinspires.ftc.teamcode.customHardware.servos.AxonServo
import org.firstinspires.ftc.teamcode.customHardware.servos.SynchronizedServo
import org.firstinspires.ftc.teamcode.extensions.BlinkExtensions.blinkFrom
import org.firstinspires.ftc.teamcode.extensions.BlinkExtensions.setPatternCo
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.initServo
import org.firstinspires.ftc.teamcode.utilClass.CameraLock
import org.firstinspires.ftc.teamcode.utilClass.ServoFunc
import org.firstinspires.ftc.teamcode.utilClass.storage.GameStorage
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.ServoUtil
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.ServoUtil.pivotHigh
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.ServoUtil.pivotLow
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.ServoUtil.pivotMid


class ScoringSubsystem(
    ahwMap: HardwareMap,
    private val auto: Boolean,
    private val armSubsystem: ArmSubsystem
) {
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
        AUTO,
        IDLE,
    }

    private var rotateServo: AxonServo
    private var rotateState: RotateState = RotateState.IDLE

    var blink: RevBlinkinLedDriver

    var targetLock: CameraLock = CameraLock.empty()

    init {
        initializeProcessor(GameStorage.alliance, PROCESSORS.TARGET_LOCK, ahwMap, "Webcam 1", true)
        claw = initServo(ahwMap, "claw")
        pitchServo = SynchronizedServo(ahwMap, "pitchServo", true)
        rotateServo = AxonServo(ahwMap, "rotateServo")
        blink = ahwMap.get(RevBlinkinLedDriver::class.java, "blink")
        setup()
    }

    fun updateLock() {
        targetLock = targetLockProcessor.cameraLock
    }

    fun updateBlink() {
        blink.setPatternCo(targetLock.color.toColor().blinkFrom())
    }


    fun update() {
        updateLock()
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

    fun setRotateAuto() {
        rotateState = RotateState.AUTO
    }

    fun setRotateIdle() {
        rotateState = RotateState.IDLE
    }

    fun telemetry(telemetry: Telemetry) {
        telemetry.addData("Scoring Subsystem", "")
        rotateServo.telemetry(telemetry)
        targetLockProcessor.telemetry(telemetry)
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
                pitchServo.setPose(pivotHigh)
                pitchState = PitchState.IDLE
                setRotateIdle()
            }

            PitchState.LOW -> {
                pitchServo.setPose(pivotLow)
                pitchState = PitchState.IDLE
                setRotateIdle()
            }

            PitchState.MED -> {
                pitchServo.setPose(pivotMid)
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
                blink.setPatternCo()
                rotateState = RotateState.IDLE
            }

            RotateState.CENTER -> {
                rotateServo.setPosition(ServoUtil.rotateCenter)
                blink.setPatternCo()
                rotateState = RotateState.IDLE
            }

            RotateState.IDLE -> {
            }

            RotateState.AUTO -> {
                rotateServo.setPosition(targetLock.angle)
                updateBlink()
            }
        }
    }

    fun setup() {
        setPitchHigh()
        closeClaw()
        setRotateCenter()
        update()
        if (auto) {
            blink.setPatternCo(GameStorage.alliance.toBinary2().toColor().blinkFrom())
        } else {
            blink.setPatternCo()
        }
    }

    fun specimenRotate(pangle: Double) {
        val angle2 = 90 - pangle
        val angleSend = (angle2 - 30)
        pitchServo.setPose(angleSend)
    }

    private fun autoLevel() {
        val angle =
            armSubsystem.degreePerTick * armSubsystem.pitchEncoder.currentPosition.toDouble()
        val coAng = Range.clip(angle + 80, -60.0, 80.0)
        pitchServo.setPose(coAng)
    }

    class ServoActions(
        private val funcs: List<Runnable>,
        private val scoringSubsystem: ScoringSubsystem
    ) :
        Action {
        override fun run(p: TelemetryPacket): Boolean {
            funcs.forEach(Runnable::run)
            scoringSubsystem.update()
            return false
        }
    }

    fun servoAction(funcs: List<Runnable>): Action {
        return ServoActions(funcs, this)
    }
}