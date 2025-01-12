package org.firstinspires.ftc.teamcode.subsystems.gameSpecific

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance
import org.firstinspires.ftc.teamcode.customHardware.servos.AxonServo
import org.firstinspires.ftc.teamcode.customHardware.servos.SynchronizedServo
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.initServo
import org.firstinspires.ftc.teamcode.utilClass.CameraLock
import org.firstinspires.ftc.teamcode.utilClass.ServoFunc
import org.firstinspires.ftc.teamcode.utilClass.storage.GameStorage
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.ServoUtil
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.ServoUtil.pivotHigh
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.ServoUtil.pivotLow
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.ServoUtil.pivotMid
import kotlin.math.abs


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
        IDLE,
    }

    private var rotateServo: AxonServo
    private var rotateState: RotateState = RotateState.IDLE

    private lateinit var limelight3A: Limelight3A

    init {
        claw = initServo(ahwMap, "claw")
        pitchServo = SynchronizedServo(ahwMap, "pitchServo", true)
        rotateServo = AxonServo(ahwMap, "rotateServo")
        if (auto) {
            limelight3A = ahwMap.get(Limelight3A::class.java, "limelight")
            limelight3A.start()
            limelight3A.pipelineSwitch(1)
            val use_blue = if (GameStorage.alliance == Alliance.BLUE) {
                1
            } else {
                0
            }
            limelight3A.updatePythonInputs(use_blue.toDouble(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            setup()
        }
    }

    var llPose = 0.0
    fun getLimeLightResult() {
        val result = limelight3A.latestResult
        val pythonOutputs: DoubleArray = result.getPythonOutput()
        var angle: Double = 0.0
        if (pythonOutputs.isNotEmpty()) {
            angle = pythonOutputs[6]
            llPose = angle
        }
    }

    fun update() {
        getLimeLightResult()
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

    fun setRotateIdle() {
        rotateState = RotateState.IDLE
    }

    fun telemetry(telemetry: Telemetry) {
        telemetry.addData("Scoring Subsystem", "")
        rotateServo.telemetry(telemetry)
//        telemetry.addData("angle", getLimeLightResult())
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
            }

            PitchState.LOW -> {
                pitchServo.setPose(pivotLow)
                pitchState = PitchState.IDLE
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
                rotateState = RotateState.IDLE
            }

            RotateState.CENTER -> {
                rotateServo.setPosition(ServoUtil.rotateCenter)
                rotateState = RotateState.IDLE
            }

            RotateState.IDLE -> {
            }
        }
    }

    fun setup() {
        setPitchHigh()
        closeClaw()
        setRotateCenter()
        update()
    }

    fun specimenRotate(pangle: Double) {
        val angle = pangle
        val angle2 = 90 - angle
        val angle3 = 180 - angle2
        val angleWrap = abs(pivotHigh) + abs(pivotLow)
        val angleSend = (angle2 - 30)
        pitchServo.setPose(angleSend)
    }

    private fun lockRotate(lock: CameraLock) {
        val angle = lock.angle
        val correctedAngle = if (angle / abs(angle) == -1.0) {
            90 + angle
        } else {
            angle
        }
        rotateServo.setPosition((correctedAngle / 5) * 5)
    }

    private fun autoLevel() {
        val angle =
            armSubsystem.degreePerTick * armSubsystem.pitchEncoder.currentPosition.toDouble()
        val coAng = Range.clip(angle + 80, -60.0, 80.0)
        pitchServo.setPose(coAng)
    }

    class ServoActions(private val funcs: List<Runnable>, val scoringSubsystem: ScoringSubsystem) :
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