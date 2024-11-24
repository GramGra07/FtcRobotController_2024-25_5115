package org.firstinspires.ftc.teamcode.subsystems.humanInput

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.extensions.GamepadExtensions
import org.firstinspires.ftc.teamcode.extensions.GamepadExtensions.buttonJustPressed
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.ArmSubsystem
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.ScoringSubsystem
import org.gentrifiedApps.statemachineftc.SequentialRunSM

object Operators {

    @JvmField
    var deadZone = 0.015

    fun bindOtherButtons(
        myOpMode: OpMode,
        scoringSubsystem: ScoringSubsystem,
        armSubsystem: ArmSubsystem
    ) {
        // "Camden", "Grady", "Michael","Graden", "Delaney", "Child"
        val otherControls = Drivers.others
        val currOther = Drivers.currOther
        if (currOther === otherControls[0]) { //Camden
            if (myOpMode.gamepad2.buttonJustPressed(
                    GamepadExtensions.PushButtons.LEFT_BUMPER,
                    2
                )
            ) {
                scoringSubsystem.openClaw()
            } else if (myOpMode.gamepad2.buttonJustPressed(
                    GamepadExtensions.PushButtons.RIGHT_BUMPER,
                    2
                )
            ) {
                scoringSubsystem.closeClaw()
            }

            if (myOpMode.gamepad2.buttonJustPressed(GamepadExtensions.PushButtons.TRIANGLE, 2)) {
                scoringSubsystem.setRotateCenter()
            } else if (myOpMode.gamepad2.buttonJustPressed(
                    GamepadExtensions.PushButtons.SQUARE,
                    2
                )
            ) {
                scoringSubsystem.setRotateLeft()
            
            } else {
                scoringSubsystem.setRotateIdle()
            }

            if (myOpMode.gamepad2.buttonJustPressed(GamepadExtensions.PushButtons.DPAD_UP, 2)) {
                scoringSubsystem.setPitchHigh()
            } else if (myOpMode.gamepad2.buttonJustPressed(
                    GamepadExtensions.PushButtons.DPAD_DOWN,
                    2
                )
            ) {
                scoringSubsystem.setPitchLow()
            } else if (myOpMode.gamepad2.buttonJustPressed(
                    GamepadExtensions.PushButtons.DPAD_RIGHT,
                    2
                )
            ) {
                scoringSubsystem.setPitchMed()
            } else if (myOpMode.gamepad2.buttonJustPressed(
                    GamepadExtensions.PushButtons.DPAD_LEFT,
                    2
                )
            ) {
                scoringSubsystem.setPitchAuto()
            } else {
                scoringSubsystem.setPitchIdle()
            }

            if (myOpMode.gamepad2.left_stick_y > 0) {
                armSubsystem.pitchIdle()
                armSubsystem.setPowerPitch(
                    -myOpMode.gamepad2.left_stick_y.toDouble(),
                    0.0
                )
            } else if (myOpMode.gamepad2.left_stick_y < 0) {
                armSubsystem.pitchIdle()
                armSubsystem.setPowerPitch(
                    -myOpMode.gamepad2.left_stick_y.toDouble(),
                    armSubsystem.maxPitchTicks.toDouble()
                )
            } else {
                armSubsystem.stopPitch()
            }

            if (myOpMode.gamepad2.right_stick_y > 0) {
                armSubsystem.extendIdle()
                armSubsystem.setPowerExtend(
                    -myOpMode.gamepad2.right_stick_y.toDouble(),
                    0.0
                )
            } else if (myOpMode.gamepad2.right_stick_y < 0) {
                armSubsystem.extendIdle()
                armSubsystem.setPowerExtend(
                    -myOpMode.gamepad2.right_stick_y.toDouble(),
                    armSubsystem.maxExtendTicks.toDouble()
                )
            } else {
                armSubsystem.stopExtend()
            }

        }
    }

    enum class DriverAidState {
        moveClaw,
        moveArm,
        moveScoring,
    }

    val driverAidSM = SequentialRunSM.Builder<DriverAidState>()
        .state(DriverAidState.moveClaw)
        .onEnter(
            DriverAidState.moveClaw
        ) {
            //move claw
        }
        .transition(DriverAidState.moveArm) { true }
        .state(DriverAidState.moveArm)
        .onEnter(
            DriverAidState.moveArm
        ) {
            //move arm
        }
        .transition(DriverAidState.moveScoring) { true }
        .state(DriverAidState.moveScoring)
        .onEnter(
            DriverAidState.moveScoring
        ) {
            //move scoring
        }
        .transition(DriverAidState.moveClaw) { true }
        .build()

    fun driverAid() {
        if (driverAidSM.isStarted) {
            driverAidSM.update()
        } else {
            driverAidSM.start()
        }
    }
}
