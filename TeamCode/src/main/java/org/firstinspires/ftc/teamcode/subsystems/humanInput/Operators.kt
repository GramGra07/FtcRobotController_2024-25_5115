package org.firstinspires.ftc.teamcode.subsystems.humanInput

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.extensions.GamepadExtensions
import org.firstinspires.ftc.teamcode.extensions.GamepadExtensions.buttonJustPressed
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.ArmSubsystem
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.ScoringSubsystem
import org.gentrifiedApps.statemachineftc.ParallelRunSM

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
            }

            if (myOpMode.gamepad2.circle) {
                if (!driverAidSM.isStarted) {
                    driverAidSM.start()
                } else {
                    driverAidSM.update()
                }
            } else {
                if (driverAidSM.isStarted) {
                    driverAidSM.stop()
                }
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
        stop
    }

    lateinit var driverAidSM: ParallelRunSM<DriverAidState>
    fun initSM(scoringSubsystem: ScoringSubsystem, armSubsystem: ArmSubsystem) {
        driverAidSM = ParallelRunSM.Builder<DriverAidState>()
            .state(DriverAidState.moveClaw)
            .onEnter(
                DriverAidState.moveClaw
            ) {
                scoringSubsystem.closeClaw()
            }
            .state(DriverAidState.moveArm)
            .onEnter(
                DriverAidState.moveArm
            ) {
                armSubsystem.autoExtend(0.0)
            }
            .state(DriverAidState.moveScoring)
            .onEnter(
                DriverAidState.moveScoring
            ) {
                scoringSubsystem.setPitchHigh()
            }
            .stopRunning(DriverAidState.stop,
                { armSubsystem.extendMotor.currentPosition.toDouble() < 100 })
            .build(false, 0)
    }
}
