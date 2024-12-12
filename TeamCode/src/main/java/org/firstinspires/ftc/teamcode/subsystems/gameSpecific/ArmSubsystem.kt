package org.firstinspires.ftc.teamcode.subsystems.gameSpecific

import com.arcrobotics.ftclib.controller.PIDFController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.customHardware.sensors.DualEncoder
import org.firstinspires.ftc.teamcode.extensions.MotorExtensions.initMotor
import org.firstinspires.ftc.teamcode.utilClass.MathFunctions
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.PIDVals
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.PIDVals.pitchTarget
import kotlin.math.cos

class ArmSubsystem(ahwMap: HardwareMap) {

    enum class ExtendState {
        PID,
        MANUAL,
        STOPPED,
        IDLE,
        AUTO
    }

    enum class PitchState {
        PID, MANUAL, STOPPED,
        IDLE,
    }

    var usePIDFp = true
    private var pitchMotor: DcMotorEx
    private var pitchMotor2: DcMotorEx
    private var pitchState: PitchState = PitchState.IDLE
    private var pPower: Double = 0.0
    private var pMax = 0.8
    private var pMin = -1.0
    private var pitchPIDF: PIDFController = PIDFController(0.0, 0.0, 0.0, 0.0)
    val pitchBottom = 100.0

    private var ticksPer90 = 2048.0

    val ticksPerDegree: Double = 90.0 / ticksPer90
    val ticksPerDegreeFunc: Double = ticksPer90 / 90.0
    val maxPitchTicks = 2600

    private fun pAngle(ticks: Double): Double {
        return (ticks * ticksPerDegree)
    }

    var usePIDFe = false
    private var extendMotor: DcMotorEx
    private var extendMotor2: DcMotorEx
    private var extendState: ExtendState = ExtendState.IDLE
    private var ePower: Double = 0.0
    private var eMax = 1.0
    private var eMin = -1.0
    private var extendPIDF: PIDFController = PIDFController(0.0, 0.0, 0.0, 0.0)
    private var extendTarget = 0

    private var extendEncoder: DualEncoder
    var pitchEncoder: DcMotorEx

    private val extendGR = 8.3521
    private val ticksPerInchExtend = (28 * extendGR) / (1.37795 * Math.PI)
    private val maxExtendIn = 33.0
    private val maxExtendTicksTOTAL = maxExtendIn * ticksPerInchExtend
    var maxExtendTicks = maxExtendTicksTOTAL


    init {
        pitchState = if (usePIDFp) {
            PitchState.PID
        } else {
            PitchState.MANUAL
        }
        extendState = if (usePIDFe) {
            ExtendState.PID
        } else {
            ExtendState.MANUAL
        }
        pitchMotor = initMotor(ahwMap, "pitchMotor", DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        pitchMotor2 = initMotor(ahwMap, "pitchMotor2", DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        extendMotor = initMotor(ahwMap, "extendMotor", DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        extendMotor2 = initMotor(ahwMap, "extendMotor2", DcMotor.RunMode.RUN_WITHOUT_ENCODER)

        extendEncoder = DualEncoder(ahwMap, "extendMotor2", "extendMotor", "Arm Extend")
        pitchEncoder = initMotor(ahwMap, "motorFrontLeft", DcMotor.RunMode.RUN_WITHOUT_ENCODER)

        pitchMotor.direction = DcMotorSimple.Direction.REVERSE
        pitchMotor2.direction = DcMotorSimple.Direction.REVERSE

        updatePID()
    }

    fun setPowerExtend(target: Double, power: Double? = 0.0) {
        updatePID()
        ePower = if (usePIDFe) {
            calculatePID(extendPIDF, extendEncoder.getAverage(), target)
        } else {
            Range.clip(
                power ?: 0.0,
                eMin,
                eMax
            )
        }
    }

    fun setPowerPitch(target: Double, overridePower: Double? = 0.0) {
        updatePID()
        pPower = if (usePIDFp) {
            calculatePID(pitchPIDF, pitchEncoder.currentPosition.toDouble(), target)
        } else {
            Range.clip(
                overridePower ?: 0.0,
                pMin,
                pMax
            )
        }
        pPower = Range.clip(pPower, pMin, pMax)
    }

    fun setPitchTarget(target: Double) {
        pitchTarget = target
    }

    fun setPitchTargetDegrees(degrees: Double) {
        pitchTarget = (degrees * ticksPerDegreeFunc)
    }

    fun setExtendTarget(target: Double) {
        extendTarget = target.toInt()
    }

    fun update(pitchPower: Double? = 0.0, extendPower: Double? = 0.0) {
        calculateExtendMax()
        if (usePIDFp) {
            setPowerPitch(pitchTarget)
        } else {
            setPowerPitch(0.0, pitchPower)
        }
        if (usePIDFe) {
            setPowerExtend(extendTarget.toDouble())
        } else {
            setPowerExtend(0.0, extendPower)
        }
        power()
    }

    fun power() {
        pitchMotor.power = pPower
        pitchMotor2.power = pPower
        extendMotor.power = ePower
        extendMotor2.power = ePower
    }

    private fun updatePID() {
        extendPIDF.setPIDF(
            PIDVals.extendPIDFCo.p,
            PIDVals.extendPIDFCo.i,
            PIDVals.extendPIDFCo.d,
            PIDVals.extendPIDFCo.f
        )
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

    private fun calculateExtendMax(): Double {
        val angle = pAngle(pitchEncoder.currentPosition.toDouble())
        val x = 42
        val cosAngle = cos(Math.toRadians(angle))
        val returnable = Range.clip((x / cosAngle) - 18, 0.0, maxExtendIn) * ticksPerInchExtend
        maxExtendTicks = returnable
        return returnable
    }

    fun telemetry(telemetry: Telemetry) {
        telemetry.addData("Arm Subsystem", "")
        pitchEncoder.telemetry(telemetry)
        extendEncoder.telemetry(telemetry)
        telemetry.addData("pAngle", pAngle(pitchEncoder.currentPosition.toDouble()))
        telemetry.addData("extendMax (ticks)", calculateExtendMax())
    }

    fun DcMotorEx.telemetry(telemetry: Telemetry) {
        telemetry.addData("Motor", this.deviceName)
        telemetry.addData("Position", this.currentPosition)
    }

    fun isPitchAtTarget(tolerance: Double = 100.0): Boolean {
        return MathFunctions.inTolerance(
            pitchEncoder.currentPosition.toDouble(),
            pitchTarget.toDouble(),
            tolerance
        )
    }

    fun isExtendAtTarget(tolerance: Double = 100.0): Boolean {
        return MathFunctions.inTolerance(
            extendEncoder.getAverage(),
            extendTarget.toDouble(),
            tolerance
        )
    }
}