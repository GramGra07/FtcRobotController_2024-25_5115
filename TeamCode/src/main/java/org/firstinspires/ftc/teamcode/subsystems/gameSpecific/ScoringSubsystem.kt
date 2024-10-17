package org.firstinspires.ftc.teamcode.subsystems.gameSpecific

import com.arcrobotics.ftclib.controller.PIDFController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.customHardware.servos.AxonServo
import org.firstinspires.ftc.teamcode.extensions.MotorExtensions.initMotor
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.initServo
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.setPose
import org.firstinspires.ftc.teamcode.utilClass.ServoFunc
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.PIDVals
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.ServoUtil

class ScoringSubsystem(ahwMap: HardwareMap) {
    enum class ClawState {
        OPEN,
        CLOSE,
        IDLE,
    }
    private var claw: Servo

    private var clawState: ClawState = ClawState.IDLE

    init {
        claw = initServo(ahwMap, "claw")
    }

    fun update() {
        updateServos()
//        updatePID()
    }
//    fun telemetry(telemetry: Telemetry) {
//        telemetry.addData("Scoring Subsystem", "")
//        telemetry.addData("Extension Position", motorExtension.currentPosition)
//    }

    fun openClaw() {
        clawState = ClawState.OPEN
    }

    fun closeClaw() {
        clawState = ClawState.CLOSE
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
    }
}