package org.firstinspires.ftc.teamcode.subsystems.gameSpecific

import com.arcrobotics.ftclib.controller.PIDFController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.customHardware.sensors.DualEncoder
import org.firstinspires.ftc.teamcode.extensions.MotorExtensions.brake
import org.firstinspires.ftc.teamcode.extensions.MotorExtensions.initMotor
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.PIDVals
import kotlin.math.sin

class ArmSubsystem(ahwMap: HardwareMap) {

    enum class ExtendState {
        PID,
        MANUAL,
        STOPPED,
        IDLE,
    }

    enum class PitchState {
        PID, MANUAL, STOPPED,
        IDLE,
    }

    private var usePIDFp = true
    private var usePIDFe = false
    private var pitchMotor: DcMotorEx
    private var pitchMotor2: DcMotorEx
    private var pitchState: PitchState = PitchState.IDLE
    private var pPower: Double = 0.0
    private var pMax = 1.0
    private var pMin = -1.0
    private var pitchPIDF: PIDFController = PIDFController(0.0, 0.0, 0.0, 0.0)

    private fun pAngle(ticks: Double): Double {
        return (ticks / (2048 * 99.8)) * 360
    }

    private var extendMotor: DcMotorEx
    private var extendMotor2: DcMotorEx
    private var extendState: ExtendState = ExtendState.IDLE
    private var ePower: Double = 0.0
    private var eMax = 1.0
    private var eMin = -1.0
    private var extendPIDF: PIDFController = PIDFController(0.0, 0.0, 0.0, 0.0)

    private val eTicksToInch: Double = (Math.PI * 1.378) / (28.0 * 20.0)
    private fun eTicksToInch(ticks: Double): Double {
        return ticks * eTicksToInch
    }

    private var extendEncoder: DualEncoder
    var pitchEncoder: DualEncoder

    val maxExtendTicks = 1200
    val maxPitchTicks = 1700


    init {
        pitchMotor = initMotor(ahwMap, "pitchMotor", DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        pitchMotor2 = initMotor(ahwMap, "pitchMotor2", DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        extendMotor = initMotor(ahwMap, "extendMotor", DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        extendMotor2 = initMotor(ahwMap, "extendMotor2", DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        pitchMotor.brake()
        pitchMotor2.brake()
        extendMotor.brake()
        extendMotor2.brake()

        extendEncoder = DualEncoder(ahwMap, "extendMotor2", "extendMotor", "Arm Extend")
        pitchEncoder = DualEncoder(ahwMap, "pitchMotor2", "pitchMotor", "Arm Pitch", true)

        updatePID()
    }

    fun setPowerExtend(power: Double, target: Double) {
        updatePID()
        when (extendState) {
            ExtendState.PID -> {
                ePower =
                    calculatePID(extendPIDF, extendEncoder.getMost(), target)
            }

            ExtendState.MANUAL -> {
                ePower = Range.clip(
                    power,
                    eMin,
                    eMax
                )
            }

            ExtendState.STOPPED -> {
                ePower = 0.0
            }

            ExtendState.IDLE -> {
                ePower = 0.0
            }
        }
        extendMotor.power = ePower
        extendMotor2.power = ePower
    }

    fun setPowerPitch(power: Double, target: Double) {
        updatePID()
        when (pitchState) {
            PitchState.PID -> {
                pPower =
                    calculatePID(pitchPIDF, pitchEncoder.getMost(), target)
            }

            PitchState.MANUAL -> {
                pPower = Range.clip(
                    power,
                    pMin,
                    pMax
                )
            }

            PitchState.STOPPED -> {
                pPower = 0.0
            }

            PitchState.IDLE -> {
                pPower = 0.0
            }
        }
        pitchMotor.power = pPower
        pitchMotor2.power = pPower
    }

    fun update() {
        updatePID()
    }

    private fun updatePID() {
        if (extendState != ExtendState.STOPPED) {
            extendState = if (usePIDFe) {
                ExtendState.PID
            } else {
                ExtendState.MANUAL
            }
        }
        extendPIDF.setPIDF(
            PIDVals.extendPIDFCo.p,
            PIDVals.extendPIDFCo.i,
            PIDVals.extendPIDFCo.d,
            PIDVals.extendPIDFCo.f
        )
        if (pitchState != PitchState.STOPPED) {// not stopped as called by code
            pitchState = if (usePIDFp) {// using the PIDF
                PitchState.PID
            } else {
                PitchState.MANUAL
            }
        }
        pitchPIDF.setPIDF(
            PIDVals.pitchPIDFCo.p,
            PIDVals.pitchPIDFCo.i,
            PIDVals.pitchPIDFCo.d,
            PIDVals.pitchPIDFCo.f
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

    fun stopExtend() {
        extendState = ExtendState.STOPPED
        setPowerExtend(0.0, extendEncoder.getMost())
    }

    fun stopPitch() {
        pitchState = PitchState.STOPPED
        setPowerPitch(0.0, pitchEncoder.getMost())
    }

    private fun getTheoreticalHeight(): Double {
        val length = eTicksToInch(extendEncoder.getAverage())
        val angle = pAngle(pitchEncoder.getAverage())
        return (sin(angle) * length) + 2.87
    }

    private var ticksPer90 = 1863
    var ticksPerDegree = ticksPer90 / 90


    fun telemetry(telemetry: Telemetry) {
        telemetry.addData("Arm Subsystem", "")
        pitchEncoder.telemetry(telemetry)
        extendEncoder.telemetry(telemetry)
        telemetry.addData("Theoretical Height (in)", getTheoreticalHeight())
    }
}