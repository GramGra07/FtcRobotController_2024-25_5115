package org.firstinspires.ftc.teamcode.subsystems.gameSpecific

import com.arcrobotics.ftclib.controller.PIDFController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.customHardware.loopTime.LoopTimeController
import org.firstinspires.ftc.teamcode.extensions.MotorExtensions.initMotor
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.getColor
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.initColorSensor
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.telemetry
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.initServo
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.setPose
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.PIDVals

class FastIntakeSubsystem(ahwMap: HardwareMap) {
    enum class IntakeState {
        RUNNING,
        STOPPED,
        REVERSED,
        IDLE,
    }

    enum class ExtendState {
        PID,
        MANUAL,
        STOPPED,
        IDLE,
    }

    enum class PitchState {
        HIGH, LOW,
        IDLE,
    }

    enum class Color {
        RED, BLUE, YELLOW, NONE
    }

    private var colorSensor: NormalizedColorSensor
    var colorDetected: Color = Color.NONE

    var usePIDF = false
    private var intakeMotor: DcMotor
    private var extendMotor: DcMotor
    private var extendState: ExtendState = ExtendState.IDLE
    private var ePower: Double = 0.0
    private var eMax = 1.0
    private var eMin = -1.0
    val extendMaxTicks = 1000.0
    val extendMinTicks = 0.0
    private var extendPIDF: PIDFController = PIDFController(0.0, 0.0, 0.0, 0.0)
    private var iPower: Double = 0.0
    private var intakeState: IntakeState = IntakeState.IDLE
    private var iMax = 1.0
    private var iMin = -1.0

    private var intakeServo: Servo
    private var intakePitchState: PitchState = PitchState.IDLE

    init {
        intakeMotor = initMotor(ahwMap, "intakeMotor", DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        extendMotor = initMotor(ahwMap, "extendMotor", DcMotor.RunMode.RUN_USING_ENCODER)
        intakeServo = initServo(ahwMap, "intakePitch")
        colorSensor = initColorSensor(ahwMap, "intakeColor")
        updatePID()
    }

    fun updateColor() {
        colorDetected = colorSensor.getColor()
    }

    fun turnOnIntake() {
        intakeState = IntakeState.RUNNING
        setIntakePower()
    }

    fun turnOffIntake() {
        intakeState = IntakeState.STOPPED
        setIntakePower()
    }

    fun reverseIntake() {
        intakeState = IntakeState.REVERSED
        setIntakePower()
    }

    fun setPowerExtend(target: Double) {
//        updatePID()
        when (extendState) {
            ExtendState.PID -> {
                ePower =
                    calculatePID(extendPIDF, extendMotor.currentPosition.toDouble(), target)
            }

            ExtendState.MANUAL -> {
                ePower = Range.clip(
                    target,
                    eMin,
                    eMax
                )
            }

            ExtendState.STOPPED -> {
                ePower = 0.0
            }

            ExtendState.IDLE -> {}
        }
    }

    fun stopExtend() {
        extendState = ExtendState.STOPPED
    }

    private fun setIntakePower() {
        when (intakeState) {
            IntakeState.RUNNING -> {
                intakeMotor.power =
                    iMax
            }

            IntakeState.STOPPED -> {
                intakeMotor.power = 0.0
            }

            IntakeState.REVERSED -> {
                intakeMotor.power = iMin
            }

            IntakeState.IDLE -> {}
        }
    }

    fun update(loopTimeController: LoopTimeController) {
//        updatePID()
        updateServo()
//        loopTimeController.every(1) {
//            updateColor()
//        }
    }

    private fun updatePID() {
        extendState = if (usePIDF) {
            ExtendState.PID
        } else {
            ExtendState.MANUAL
        }
        extendPIDF.setPIDF(
            PIDVals.extendPIDFCo.p,
            PIDVals.extendPIDFCo.i,
            PIDVals.extendPIDFCo.d,
            PIDVals.extendPIDFCo.f
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

    fun idleExtend() {
        extendState = ExtendState.IDLE
    }

    fun pitchHigh() {
        intakePitchState = PitchState.HIGH
    }

    fun pitchLow() {
        intakePitchState = PitchState.LOW
    }

    private fun updateServo() {
        when (intakePitchState) {
            PitchState.HIGH -> {
                intakeServo.setPose(30.0)
                intakePitchState = PitchState.IDLE
            }

            PitchState.LOW -> {
                intakeServo.setPose(0.0)
                intakePitchState = PitchState.IDLE
            }

            PitchState.IDLE -> {}
        }
    }

    fun telemetry(telemetry: Telemetry) {
        telemetry.addData("Intake Subsystem", "")
        telemetry.addData("Intake Position", intakeMotor.currentPosition)
        telemetry.addData("Extend Position", extendMotor.currentPosition)
        telemetry.addData("Color", colorSensor.telemetry(telemetry))
    }
}