package org.firstinspires.ftc.teamcode.subsystems.gameSpecific

import com.arcrobotics.ftclib.controller.PIDFController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.extensions.MotorExtensions.initMotor
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.PIDVals

class LiftSubsystem(ahwMap: HardwareMap) {
    enum class LiftState {
        PID_1,
        MANUAL_1,
        //        PID_2,
//        MANUAL_2,
        STOPPED,
        IDLE,
    }

    private var liftState: LiftState = LiftState.IDLE
    private var motorLift: DcMotor
    private var lPower = 0.0

    private var liftPIDF = PIDFController(0.0, 0.0, 0.0, 0.0)

    private var lMin = 0.0
    private var lMax = 1.0

    private var usePIDF = true

    var maxLiftExtension = 1000.0
    var minLiftExtension = 0.0

    init {
        motorLift = initMotor(ahwMap, "liftMotor", DcMotor.RunMode.RUN_USING_ENCODER)
        idle()
    }

    fun setPower(target: Double) {
        updatePID()
        when (liftState) {
            LiftState.PID_1 -> {
                lPower = calculatePID(liftPIDF, motorLift.currentPosition.toDouble(), target)
            }

            LiftState.MANUAL_1 -> {
                lPower = Range.clip(
                    target,
                    lMin,
                    lMax
                )
            }
//            LiftState.PID_2 -> {
//                lPower = calculatePID(liftPIDF, motorLift.currentPosition.toDouble(), input)
//            }
//            LiftState.MANUAL_2 -> {
//                lPower = Range.clip(
//                    input,
//                    lMin,
//                    lMax
//                )
//            }

            LiftState.STOPPED -> {
                lPower = 0.0
                idle()
            }

            LiftState.IDLE -> {}
        }
    }

    fun stop() {
        updatePID()
        lPower = 0.0
        liftState = LiftState.STOPPED
    }

    private fun power() {
        when (liftState) {
            LiftState.MANUAL_1,
            LiftState.PID_1 -> {
                motorLift.power = lPower
            }

            LiftState.STOPPED -> {
                motorLift.power = 0.0
            }

            LiftState.IDLE -> {}
        }
    }

    fun update() {
        updatePID()
        power()
    }

    private fun updatePID() {
        liftState = if (usePIDF) {
            LiftState.PID_1
        } else {
            LiftState.MANUAL_1
        }
        liftPIDF.setPIDF(
            PIDVals.liftPIDFCo.p,
            PIDVals.liftPIDFCo.i,
            PIDVals.liftPIDFCo.d,
            PIDVals.liftPIDFCo.f
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

    fun idle() {
        liftState = LiftState.IDLE
    }

    fun telemetry(telemetry: Telemetry) {
        telemetry.addData("Lift Subsystem", "")
        telemetry.addData("Lift Position", motorLift.currentPosition)
    }
}