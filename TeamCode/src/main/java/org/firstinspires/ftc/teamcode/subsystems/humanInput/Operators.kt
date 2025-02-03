package org.firstinspires.ftc.teamcode.subsystems.humanInput

import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.extensions.BlinkExtensions.setPatternCo
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
            scoringSubsystem.shouldRotate = false
        } else if (myOpMode.gamepad2.buttonJustPressed(
                GamepadExtensions.PushButtons.SQUARE,
                2
            )
        ) {
            scoringSubsystem.setRotateLeft()
            scoringSubsystem.shouldRotate = false
        } else if (myOpMode.gamepad2.buttonJustPressed(
                GamepadExtensions.PushButtons.CIRCLE,
                2
            )
        ) {
            scoringSubsystem.setPitchMed()
            scoringSubsystem.shouldRotate = false
        } else if (myOpMode.gamepad2.buttonJustPressed(
                GamepadExtensions.PushButtons.CROSS,
                2
            )
        ) {
            scoringSubsystem.setPitchLow()
            scoringSubsystem.shouldRotate = false
        }

        if (myOpMode.gamepad2.buttonJustPressed(GamepadExtensions.PushButtons.DPAD_UP, 2)) {
            da.highSpecimen()
        } else if (myOpMode.gamepad2.buttonJustPressed(
                GamepadExtensions.PushButtons.DPAD_LEFT,
                2
            )
        ) {
            da.highBasket()
        } else if (myOpMode.gamepad2.buttonJustPressed(
                GamepadExtensions.PushButtons.DPAD_DOWN,
                2
            )
        ) {
            da.pickup()
        } else if (myOpMode.gamepad2.buttonJustPressed(
                GamepadExtensions.PushButtons.DPAD_RIGHT,
                2
            )
        ) {
            da.human()
        } else if (myOpMode.gamepad2.touchpad) {
            da.collapse()
        }

//        if (!armSubsystem.usePIDFp) {
//            armSubsystem.setPowerPitch(0.0, myOpMode.gamepad2.left_stick_y.toDouble())
//        }
//        else {
//            if (myOpMode.gamepad2.left_stick_button) {
//                armSubsystem.setPitchTarget(0.0)
//            } else if (myOpMode.gamepad2.right_stick_button) {
//                armSubsystem.setPitchTarget(600.0)
//            }
//        }
        if (myOpMode.gamepad2.left_trigger > 0.0) {
            da.lift()
        }else if (myOpMode.gamepad2.right_trigger > 0.0) {
            da.
            armSubsystem.setExtendTarget(armSubsystem.extendEncoder.getMost()-100)
        } else {
            da.firstLevelLift.restartAtBeginning()
        }

        if (myOpMode.gamepad2.right_stick_button){
            armSubsystem.pitchEncoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            armSubsystem.extendMotor.mode= DcMotor.RunMode.STOP_AND_RESET_ENCODER
            armSubsystem.extendMotor2.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            armSubsystem.pitchEncoder.mode = DcMotor.RunMode.RUN_USING_ENCODER
            armSubsystem.extendMotor.mode= DcMotor.RunMode.RUN_USING_ENCODER
            armSubsystem.extendMotor2.mode = DcMotor.RunMode.RUN_USING_ENCODER
            scoringSubsystem.blink.setPatternCo(RevBlinkinLedDriver.BlinkinPattern.GREEN)
        }


//        if (myOpMode.gamepad2.right_stick_button) {
//            armSubsystem.setPE(VarConfig.pitch, VarConfig.extend)
//        }

//        if (myOpMode.gamepad2.right_stick_button) {
//            armSubsystem.correctHeight()
//        }
        if (myOpMode.gamepad2.left_stick_button) {
            scoringSubsystem.setPitchHigh()
            scoringSubsystem.shouldRotate = false
        }

    }
}
