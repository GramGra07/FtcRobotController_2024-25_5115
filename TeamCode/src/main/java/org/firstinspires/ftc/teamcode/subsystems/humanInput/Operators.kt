package org.firstinspires.ftc.teamcode.subsystems.humanInput

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.extensions.GamepadExtensions
import org.firstinspires.ftc.teamcode.extensions.GamepadExtensions.buttonJustPressed
import org.firstinspires.ftc.teamcode.subsystems.DriverAid
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.ArmSubsystem
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.ScoringSubsystem

object Operators {

    @JvmField
    var deadZone = 0.015

    fun bindOtherButtons(
        myOpMode: OpMode,
        scoringSubsystem: ScoringSubsystem,
        armSubsystem: ArmSubsystem,
        da: DriverAid
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


            val autoLift =
                myOpMode.gamepad2.buttonJustPressed(GamepadExtensions.PushButtons.RIGHT_BUMPER, 2)
            if (autoLift) {
                da.lift()
            } else {

                if (myOpMode.gamepad2.left_stick_y > 0) {
                    armSubsystem.setPitchTarget(0.0)
                } else if (myOpMode.gamepad2.left_stick_y < 0) {
                    armSubsystem.setPitchTarget(armSubsystem.maxPitchTicks.toDouble())
                }

//            if (myOpMode.gamepad2.right_stick_y > 0) {
//                armSubsystem.setExtendTarget(0.0)
//            } else if (myOpMode.gamepad2.right_stick_y < 0) {
//                armSubsystem.setExtendTarget(armSubsystem.maxExtendTicks.toDouble())
//            } else
                if (myOpMode.gamepad2.circle) {
                    da.collapse()
                } else {
                    armSubsystem.setExtendTarget(myOpMode.gamepad2.right_trigger.toDouble() * armSubsystem.maxExtendTicks)
                }
            }

        }
    }
}
