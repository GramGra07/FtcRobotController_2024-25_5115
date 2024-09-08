package org.firstinspires.ftc.teamcode.subsystems.humanInput

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.FastIntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.ScoringSubsystem

@Config
object Operators {

    @JvmField
    var deadZone = 0.15

    private var touchPressed = false
    private var xPressed = false
    fun bindOtherButtons(
        myOpMode: OpMode,
        intakeSubsystem: FastIntakeSubsystem,
        scoringSubsystem: ScoringSubsystem,
    ) {

        // "Camden", "Grady", "Michael","Graden", "Delaney", "Child"
        val otherControls = Drivers.others
        val currOther = Drivers.currOther
        if (currOther === otherControls[0]) { //Camden
            touchPressed = myOpMode.gamepad2.touchpad

            if (myOpMode.gamepad2.left_trigger > 0.0) {
                intakeSubsystem.turnOnIntake()
            } else if (myOpMode.gamepad2.right_trigger > 0.0) {
                intakeSubsystem.reverseIntake()
            } else {
                intakeSubsystem.turnOffIntake()
            }

            if (myOpMode.gamepad2.square) {
                intakeSubsystem.pitchHigh()
            } else if (myOpMode.gamepad2.circle) {
                intakeSubsystem.pitchLow()
            }

            if (myOpMode.gamepad2.right_bumper) {
                scoringSubsystem.openClaw()
            }
            if (myOpMode.gamepad2.left_bumper) {
                scoringSubsystem.closeClaw()
            }

            if (myOpMode.gamepad2.dpad_left) {
                scoringSubsystem.pitchHigh()
            } else if (myOpMode.gamepad2.dpad_up) {
                scoringSubsystem.pitchLow()
            } else if (myOpMode.gamepad2.dpad_down) {
                scoringSubsystem.topPitchHigh()
            } else if (myOpMode.gamepad2.dpad_right) {
                scoringSubsystem.topPitchLow()
            }

            if (myOpMode.gamepad2.right_stick_y < -deadZone && intakeSubsystem.usePIDF) {
                intakeSubsystem.setPowerExtend(intakeSubsystem.extendMaxTicks)
            } else if (myOpMode.gamepad2.right_stick_y > deadZone && intakeSubsystem.usePIDF) {
                intakeSubsystem.setPowerExtend(intakeSubsystem.extendMinTicks.toDouble())
            } else {
                intakeSubsystem.stopExtend()
            }

            if (myOpMode.gamepad2.left_stick_y > deadZone && scoringSubsystem.usePIDF) {
                scoringSubsystem.setPowerE(scoringSubsystem.extensionMinTicks.toDouble())
            } else if (myOpMode.gamepad2.left_stick_y < -deadZone && scoringSubsystem.usePIDF) {
                scoringSubsystem.setPowerE(scoringSubsystem.extensionMaxTicks.toDouble())
            } else {
                scoringSubsystem.stopE()
            }

            if (myOpMode.gamepad2.cross && !xPressed && scoringSubsystem.usePIDF) {
                scoringSubsystem.usePIDF = false
            } else if (myOpMode.gamepad2.cross && !xPressed && !scoringSubsystem.usePIDF) {
                scoringSubsystem.usePIDF = true
            }
            xPressed = myOpMode.gamepad2.cross

            if (!scoringSubsystem.usePIDF) {
                scoringSubsystem.setPowerE(
                    -myOpMode.gamepad2.left_stick_y.toDouble()
                )
            }
            if (!intakeSubsystem.usePIDF){
                intakeSubsystem.setPowerExtend(
                    -myOpMode.gamepad2.right_stick_y.toDouble()
                )
            }
        }
        if (currOther === otherControls[1]) { //Grady
        }
        if (currOther === otherControls[2]) { //Michael
        }
        if (currOther === otherControls[3]) { //Graden
        }
        if (currOther === otherControls[4]) { // Delaney
        }
        if (currOther === otherControls[5]) { // Child
        }
    }
}
