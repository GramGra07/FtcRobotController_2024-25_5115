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
import kotlin.math.cos
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

    private var usePIDFp = false
    private var pitchMotor: DcMotorEx
    private var pitchMotor2: DcMotorEx
    private var pitchState: PitchState = PitchState.IDLE
    private var pPower: Double = 0.0
    private var pMax = 1.0
    private var pMin = -1.0
    private var pitchPIDF: PIDFController = PIDFController(0.0, 0.0, 0.0, 0.0)

    private fun pAngle(ticks: Double): Double {
        return (ticks * ticksPerDegreeCalc)
    }

    private var usePIDFe = false
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

    val maxExtendTicksTOTAL = 1200
    var maxExtendTicks = maxExtendTicksTOTAL
    val maxPitchTicks = 1600


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
                    calculatePID(extendPIDF, extendEncoder.getAverage(), target)
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
    }

    fun update() {
        updatePID()
        power()
        calculateExtendMax()
    }

    fun power() {
        pitchMotor.power = pPower
        pitchMotor2.power = pPower
        extendMotor.power = ePower
        extendMotor2.power = ePower
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
        pPower = 0.0
    }

    fun stopPitch() {
        pitchState = PitchState.STOPPED
        ePower = 0.0
    }

    private fun getTheoreticalHeight(): Double {
        val length = eTicksToInch(extendEncoder.getAverage())
        val angle = pAngle(pitchEncoder.getAverage())
        return (sin(angle) * length) + 2.87
    }

    private var ticksPer90 = 1863
    val ticksPerDegree = ticksPer90 / 90
    private val ticksPerDegreeCalc = 0.04839

    private val ticksPerInchExtend = 163.0

    fun pitchIdle() {
        pitchState = PitchState.IDLE
    }

    fun extendIdle() {
        extendState = ExtendState.IDLE
    }

    private fun calculateExtendMax(): Double {
        val angle = pAngle(pitchEncoder.getAverage())
        val x = 42
        val cosAngle = cos(Math.toRadians(angle))
        val returnable = Range.clip((x / cosAngle) - 18, 0.0, 44.0) * ticksPerInchExtend
        maxExtendTicks = (returnable).toInt()
        return returnable
    }

    fun telemetry(telemetry: Telemetry) {
        telemetry.addData("Arm Subsystem", "")
        pitchEncoder.telemetry(telemetry)
        extendEncoder.telemetry(telemetry)
        telemetry.addData("pAngle", pAngle(pitchEncoder.getAverage()))
        telemetry.addData("extendMax (ticks)", calculateExtendMax())
    }
}