package org.firstinspires.ftc.teamcode.subsystems.gameSpecific

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.arcrobotics.ftclib.controller.PIDFController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.customHardware.sensors.DualEncoder
import org.firstinspires.ftc.teamcode.extensions.MotorExtensions.initMotor
import org.firstinspires.ftc.teamcode.utilClass.MathFunctions
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.PIDVals
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.PIDVals.pitchFCollapse
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.PIDVals.pitchFExtend
import kotlin.math.asin
import kotlin.math.cos
import kotlin.math.sqrt

class ArmSubsystem(ahwMap: HardwareMap, auto: Boolean) {
    val pitchNegate = if (auto) -1.0 else 1.0

    val gearRatioMult = 1.249

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
    var pMax = 0.8
    private var pMin = -0.7
    private var pitchPIDF: PIDFController = PIDFController(0.0, 0.0, 0.0, 0.0)
    var pitchT: Double = 0.0
    var pitchOffset: Double = 0.0

    private var ticksPer90 = 2048.0

    val degreePerTick: Double = 90.0 / ticksPer90
    val ticksPerDegree: Double = ticksPer90 / 90.0
    val maxPitchTicks = 2600

    fun pAngle(): Double {
        return ((pitchEncoder.currentPosition.toDouble() * pitchNegate) * degreePerTick)
    }

    var usePIDFe = true
    private var extendMotor: DcMotorEx
    private var extendMotor2: DcMotorEx
    private var extendState: ExtendState = ExtendState.IDLE
    private var ePower: Double = 0.0
    var eMax = 1.0
    private var eMin = -1.0
    private var extendPIDF: PIDFController = PIDFController(0.0, 0.0, 0.0, 0.0)
    var extendTarget = 0

    var extendEncoder: DualEncoder
    var pitchEncoder: DcMotorEx

    private val maxExtendIn = 32.5
    private val maxExtendTicksTOTAL = 2200 * gearRatioMult
    private val ticksPerInchExtend = (maxExtendTicksTOTAL / maxExtendIn)
    var maxExtendTicks = maxExtendTicksTOTAL

//    var distanceSensor: DistanceSensor


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

        extendEncoder = DualEncoder(ahwMap, "extendMotor2", "extendMotor", "Arm Extend", false)
        pitchEncoder = initMotor(ahwMap, "motorFrontRight", DcMotor.RunMode.RUN_WITHOUT_ENCODER)

//        pitchMotor.direction = DcMotorSimple.Direction.REVERSE
//        pitchMotor2.direction = DcMotorSimple.Direction.REVERSE
//        extendMotor.direction = DcMotorSimple.Direction.REVERSE
//        extendMotor2.direction = DcMotorSimple.Direction.REVERSE

        updatePID()

//        distanceSensor = ahwMap.get(DistanceSensor::class.java, "distanceSensor")
    }

//    fun findOffset() {
//        val target = 7.0
//        val dError = target - distanceSensor.getDistance(DistanceUnit.INCH)
//        val h = extendEncoder.getMost() * inPerTickExtend
//        val angle = asin(dError / h)
//        val offset = Math.toDegrees(angle) * ticksPerDegree
////        pitchOffset = offset
//    }
//
//    fun correctHeight() {
//        val targetDistance = 6.1
//        val dError = targetDistance - distanceSensor.getDistance(DistanceUnit.INCH)
//        val h = extendEncoder.getMost() * inPerTickExtend
//        val angle = asin(dError / h)
//        val newTarget = (Math.toDegrees(angle) * ticksPerDegree) + pitchT
//        setPitchTarget(newTarget)
//
//    }

    fun setPowerExtend(target: Double, power: Double? = 0.0) {
        ePower = //if (usePIDFe) {
            calculatePID(extendPIDF, extendEncoder.getMost(), target)
//        } else {
//            Range.clip(
//                power ?: 0.0,
//                eMin,
//                eMax
//            )
//        }
    }

    fun setPowerPitch(target: Double, overridePower: Double? = 0.0) {
        pPower = //if (usePIDFp) {
            calculatePID(pitchPIDF, pitchEncoder.currentPosition.toDouble() * pitchNegate, target)
//        } else {
//            Range.clip(
//                overridePower ?: 0.0,
//                pMin,
//                pMax
//            )
//        }
        pPower = Range.clip(pPower, pMin, pMax)
    }

    fun setPitchTarget(target: Double) {
        pitchT = target
    }

    fun setPitchTargetDegrees(degrees: Double) {
        setPitchTarget(degrees * ticksPerDegree)
    }

    fun setExtendTarget(target: Double) {
        extendTarget = (target).toInt()
    }

    fun setExtendTargetIn(inches: Double) {
        extendTarget = (inches * ticksPerInchExtend).toInt()
    }

    fun update(
        pitchPower: Double? = 0.0,
        extendPower: Double? = 0.0,
        overridePower: Boolean = false
    ) {
        updatePID()
        calculateExtendMax()
        if (usePIDFp && !overridePower) {
            setPowerPitch(pitchT)
        } else {
            setPowerPitch(0.0, pitchPower)
        }
        if (usePIDFe && !overridePower) {
            setPowerExtend(extendTarget.toDouble())
        } else {
            setPowerExtend(0.0, extendPower)
        }
        power()
        pMax = 0.8
        PIDVals.pitchPIDFCo.d = 0.00005
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
//        extendPIDF.setPIDF(0.013, 0.0, 0.0000, 0.0)
        val fTick = (pitchFExtend - pitchFCollapse) / maxExtendTicksTOTAL
        val pitchF = (fTick * extendEncoder.getMost()) + pitchFCollapse
//        val pitchF = PIDVals.pitchPIDFCo.f
        pitchPIDF.setPIDF(
            PIDVals.pitchPIDFCo.p,
            PIDVals.pitchPIDFCo.i,
            PIDVals.pitchPIDFCo.d,
            pitchF
        )
//        pitchPIDF.setPIDF(0.0017, 0.0, 0.00005, pitchF)
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
        val angle = pAngle()
        val x = 42
        val cosAngle = cos(Math.toRadians(angle))
        val returnable = Range.clip((x / cosAngle) - 18, 0.0, maxExtendIn) * ticksPerInchExtend
        maxExtendTicks = returnable
        return returnable
    }

    fun telemetry(telemetry: Telemetry) {
        telemetry.addData("Arm Subsystem", "")
//        telemetry.addData("Dist", distanceSensor.getDistance(DistanceUnit.INCH))
        pitchEncoder.telemetry(telemetry)
        extendEncoder.telemetry(telemetry)
        telemetry.addData("eTarget", extendTarget)
        telemetry.addData("pTarget", pitchT)
        telemetry.addData("pAngle", pAngle())
        telemetry.addData("extendMax (ticks)", calculateExtendMax())
    }

    fun DcMotorEx.telemetry(telemetry: Telemetry) {
        telemetry.addData("Motor", this.deviceName)
        telemetry.addData("Position", this.currentPosition)
    }

    fun isPitchAtTarget(tolerance: Double = 100.0): Boolean {
        return MathFunctions.inTolerance(
            pitchEncoder.currentPosition.toDouble() * pitchNegate,
            pitchT.toDouble(),
            tolerance
        )
    }

    fun isExtendAtTarget(tolerance: Double = 100.0): Boolean {
        return MathFunctions.inTolerance(
            extendEncoder.getMost(),
            extendTarget.toDouble(),
            tolerance
        )
    }

    fun bothAtTarget(tolerance: Double = 100.0): Boolean {
        return isPitchAtTarget(tolerance) && isExtendAtTarget(tolerance)
    }

    fun isEnded(tolerance: Double = 100.0): Boolean {
        return bothAtTarget(tolerance) && secondActionRun
    }

    private var pitchHitPosition = false
    private var extendHitPosition = false
    var secondActionRun = false
    fun resetHitPosition(shouldReset: Boolean = true) {
        pitchHitPosition = false
        extendHitPosition = false
        secondActionRun = false
    }

    fun setPE(p: Double, e: Double, pitchFirst: Boolean? = null) {

        when (pitchFirst) {
            null -> {
                // Default behavior: Set pitch and extend simultaneously
                if (!pitchHitPosition) setPitchTarget(p)
                if (!extendHitPosition) setExtendTarget(e)
            }

            true -> {
                // Set pitch first, then extend
                if (!pitchHitPosition) {
                    setPitchTarget(p)
                }
                if (pitchHitPosition) {
                    setExtendTarget(e)
                    secondActionRun = true
                }
            }

            false -> {
                // Set extend first, then pitch
                if (!extendHitPosition) {
                    setExtendTarget(e)
                }
                if (extendHitPosition) {
                    setPitchTarget(p)
                    secondActionRun = true
                }
            }
        }
        // Update hit positions
        pitchHitPosition = isPitchAtTarget(300.0)
        extendHitPosition = isExtendAtTarget(200.0)
    }


    fun setHeight(
        height: Double,
        length: Double,
        compensateForClaw: Boolean = false,
        compensateForPivot: Boolean = false
    ) {
        val correctedHeight =
            height - (if (compensateForClaw) 5.3 else 0.0) - if (compensateForPivot) 3.71 else 0.0
        val h = sqrt((correctedHeight * correctedHeight) + (length * length))
        val angle = asin(correctedHeight / h)
        val correctedH = h - 16.0
        setExtendTargetIn(correctedH)
        setPitchTargetDegrees(Math.toDegrees(angle))
    }

    class PEAction(private val funcs: List<Runnable>, private val armSubsystem: ArmSubsystem) :
        Action {
        override fun run(packet: TelemetryPacket): Boolean {
            funcs.forEach(Runnable::run)
            armSubsystem.update()
            packet.put("pTarget", armSubsystem.pitchT)
            packet.put(
                "p",
                armSubsystem.pitchEncoder.currentPosition.toDouble()
            )
            packet.put("eTarget", armSubsystem.extendTarget)
            packet.put("e", armSubsystem.extendEncoder.getMost())
            return !armSubsystem.bothAtTarget()
        }
    }

    fun setPEAction(funcs: List<Runnable>): Action {
        return PEAction(funcs, this)
    }
}