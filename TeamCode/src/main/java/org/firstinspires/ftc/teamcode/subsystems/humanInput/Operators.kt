package org.firstinspires.ftc.teamcode.subsystems.humanInput

import com.qualcomm.robotcore.eventloop.opmode.OpMode

object Operators {

    @JvmField
    var deadZone = 0.15

    private var touchPressed = false
    private var xPressed = false
    fun bindOtherButtons(
        myOpMode: OpMode,
//        intakeSubsystem: FastIntakeSubsystem,
//        scoringSubsystem: ScoringSubsystem,
    ) {

        // "Camden", "Grady", "Michael","Graden", "Delaney", "Child"
        val otherControls = Drivers.others
        val currOther = Drivers.currOther
        if (currOther === otherControls[0]) { //Camden
            touchPressed = myOpMode.gamepad2.touchpad

//            if (myOpMode.gamepad2.left_trigger > 0.0) {
//                intakeSubsystem.turnOnIntake()
//            } else if (myOpMode.gamepad2.right_trigger > 0.0) {
//                intakeSubsystem.reverseIntake()
//            } else {
//                intakeSubsystem.turnOffIntake()
//            }
//
//            if (myOpMode.gamepad2.buttonJustPressed(GamepadExtensions.PushButtons.SQUARE,2)) {
//                intakeSubsystem.pitchHigh()
//            } else if (myOpMode.gamepad2.buttonJustPressed(GamepadExtensions.PushButtons.CIRCLE,2)) {
//                intakeSubsystem.pitchLow()
//            }
//
//            if (myOpMode.gamepad2.buttonJustPressed(GamepadExtensions.PushButtons.RIGHT_BUMPER,2)) {
//                scoringSubsystem.openClaw()
//            }
//            if (myOpMode.gamepad2.buttonJustPressed(GamepadExtensions.PushButtons.LEFT_BUMPER,2)) {
//                scoringSubsystem.closeClaw()
//            }
//
//            if (myOpMode.gamepad2.buttonJustPressed(GamepadExtensions.PushButtons.DPAD_DOWN,2)) {
//                scoringSubsystem.pitchHigh()
//            } else if (myOpMode.gamepad2.buttonJustPressed(GamepadExtensions.PushButtons.DPAD_UP,2)) {
//                scoringSubsystem.pitchLow()
//            }
//            if (myOpMode.gamepad2.buttonJustPressed(GamepadExtensions.PushButtons.DPAD_LEFT,2)) {
//                scoringSubsystem.topPitchForward()
//            } else if (myOpMode.gamepad2.buttonJustPressed(GamepadExtensions.PushButtons.DPAD_RIGHT,2)) {
//                scoringSubsystem.topPitchReverse()
//            } else {
//                scoringSubsystem.idleTopPitch()
//            }
//
//            if (myOpMode.gamepad2.right_stick_y < -deadZone && intakeSubsystem.usePIDF) {
//                intakeSubsystem.setPowerExtend(intakeSubsystem.extendMaxTicks)
//            } else if (myOpMode.gamepad2.right_stick_y > deadZone && intakeSubsystem.usePIDF) {
//                intakeSubsystem.setPowerExtend(intakeSubsystem.extendMinTicks)
//            } else {
//                intakeSubsystem.stopExtend()
//            }
//
//            if (myOpMode.gamepad2.left_stick_y > deadZone && scoringSubsystem.usePIDF) {
//                scoringSubsystem.setPowerE(scoringSubsystem.extensionMinTicks.toDouble())
//            } else if (myOpMode.gamepad2.left_stick_y < -deadZone && scoringSubsystem.usePIDF) {
//                scoringSubsystem.setPowerE(scoringSubsystem.extensionMaxTicks.toDouble())
//            } else {
//                scoringSubsystem.stopE()
//            }
//
//            if (myOpMode.gamepad2.cross && !xPressed && scoringSubsystem.usePIDF) {
//                scoringSubsystem.usePIDF = false
//            } else if (myOpMode.gamepad2.cross && !xPressed && !scoringSubsystem.usePIDF) {
//                scoringSubsystem.usePIDF = true
//            }
//            xPressed = myOpMode.gamepad2.cross
//
//            if (!scoringSubsystem.usePIDF) {
//                scoringSubsystem.setPowerE(
//                    -myOpMode.gamepad2.left_stick_y.toDouble()
//                )
//            }
//            if (!intakeSubsystem.usePIDF) {
//                intakeSubsystem.setPowerExtend(
//                    -myOpMode.gamepad2.right_stick_y.toDouble()
//                )
//            }
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
