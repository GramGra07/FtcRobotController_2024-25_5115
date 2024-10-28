package org.firstinspires.ftc.teamcode.subsystems.gameSpecific

import com.arcrobotics.ftclib.controller.PIDFController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.customHardware.sensors.DualEncoder
import org.firstinspires.ftc.teamcode.extensions.MotorExtensions.initMotor
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.PIDVals

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

    var usePIDF = false
    private var pitchMotor: DcMotor
    private var pitchMotor2: DcMotor
    private var pitchState: PitchState = PitchState.IDLE
    private var pPower: Double = 0.0
    private var pMax = 1.0
    private var pMin = -1.0
    private var pitchPIDF: PIDFController = PIDFController(0.0, 0.0, 0.0, 0.0)

    private var extendMotor: DcMotor
    private var extendMotor2: DcMotor
    private var extendState: ExtendState = ExtendState.IDLE
    private var ePower: Double = 0.0
    private var eMax = 1.0
    private var eMin = -1.0
    private var extendPIDF: PIDFController = PIDFController(0.0, 0.0, 0.0, 0.0)

    private var extendEncoder: DcMotor
    private var pitchEncoder: DualEncoder

    val maxExtendTicks = 100
    val maxPitchTicks = 100


    init {
        pitchMotor = initMotor(ahwMap, "pitchMotor", DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        pitchMotor2 = initMotor(ahwMap, "pitchMotor2", DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        extendMotor = initMotor(ahwMap, "extendMotor", DcMotor.RunMode.RUN_USING_ENCODER)
        extendMotor2 = initMotor(ahwMap, "extendMotor2", DcMotor.RunMode.RUN_USING_ENCODER)
        extendEncoder = extendMotor
        pitchEncoder = DualEncoder(ahwMap, "pitchMotor", "pitchMotor2", "armSubsystem")
        updatePID()
    }

    fun setPowerExtend(target: Double) {
        when (extendState) {
            ExtendState.PID -> {
                ePower =
                    calculatePID(extendPIDF, extendEncoder.currentPosition.toDouble(), target)
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

    fun setPowerPitch(target: Double) {
        when (pitchState) {
            PitchState.PID -> {
                pPower =
                    calculatePID(pitchPIDF, pitchEncoder.getAverage(), target)
            }

            PitchState.MANUAL -> {
                pPower = Range.clip(
                    0.0,
                    pMin,
                    pMax
                )
            }

            PitchState.STOPPED -> {
                pPower = 0.0
            }

            PitchState.IDLE -> {}
        }
    }

    fun update() {
        power()
    }

    private fun power() {
        when (extendState) {
            ExtendState.PID,
            ExtendState.MANUAL -> {
                extendMotor.power = ePower
                extendMotor2.power = ePower
            }

            ExtendState.STOPPED,
            ExtendState.IDLE -> {
                extendMotor.power = 0.0
                extendMotor2.power = 0.0
            }
        }
        when (pitchState) {
            PitchState.PID,
            PitchState.MANUAL -> {
                pitchMotor.power = pPower
                pitchMotor2.power = pPower
            }

            PitchState.STOPPED,
            PitchState.IDLE -> {
                pitchMotor.power = 0.0
                pitchMotor2.power = 0.0
            }
        }
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
        pitchState = if (usePIDF) {
            PitchState.PID
        } else {
            PitchState.MANUAL
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

    fun idleExtend() {
        extendState = ExtendState.IDLE
    }

    fun idlePitch() {
        pitchState = PitchState.IDLE
    }

    fun stopExtend() {
        extendState = ExtendState.STOPPED
    }

    fun stopPitch() {
        pitchState = PitchState.STOPPED
    }


    fun telemetry(telemetry: Telemetry) {
        telemetry.addData("Intake Subsystem", "")
        telemetry.addData("Pitch Encoder", pitchEncoder.getAverage())
        telemetry.addData("Extend Encoder", extendEncoder.currentPosition)
    }
}