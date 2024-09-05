package org.firstinspires.ftc.teamcode.subsystems.humanInput

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.ClawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.ExtendoSubsystem

@Config
object Operators {

    @JvmField
    var deadZone = 0.15

    private var touchPressed = false
    private var xPressed = false
    fun bindOtherButtons(
        myOpMode: OpMode,
        clawSubsystem: ClawSubsystem,
        extendoSubsystem: ExtendoSubsystem,
    ) {

        // "Camden", "Grady", "Michael","Graden", "Delaney", "Child"
        val otherControls = Drivers.others
        val currOther = Drivers.currOther
        if (currOther === otherControls[0]) { //Camden
            touchPressed = myOpMode.gamepad2.touchpad
            if (myOpMode.gamepad2.right_bumper) {
                clawSubsystem.closeLeft()
            }
            if (myOpMode.gamepad2.left_bumper) {
                clawSubsystem.closeRight()
            }
            if (myOpMode.gamepad2.right_trigger > 0) {
                clawSubsystem.openLeft()
            }
            if (myOpMode.gamepad2.left_trigger > 0) {
                clawSubsystem.openRight()
            }

            if (myOpMode.gamepad2.dpad_left) {
                clawSubsystem.flipBack()
            } else if (myOpMode.gamepad2.dpad_up) {
                clawSubsystem.flipHigh()
            } else if (myOpMode.gamepad2.dpad_down) {
                clawSubsystem.flipZero()
            }
            if (myOpMode.gamepad2.right_stick_y < -deadZone && extendoSubsystem.usePIDF) {
                extendoSubsystem.setPowerR(extendoSubsystem.maxRotationTicks.toDouble())
            } else if (myOpMode.gamepad2.right_stick_y > deadZone && extendoSubsystem.usePIDF) {
                extendoSubsystem.setPowerR(extendoSubsystem.minRotationTicks.toDouble())
            } else {
                extendoSubsystem.stopR()
                extendoSubsystem.idleR()
            }
            if (myOpMode.gamepad2.cross && !xPressed && extendoSubsystem.usePIDF) {
                extendoSubsystem.usePIDF = false
            } else if (myOpMode.gamepad2.cross && !xPressed && !extendoSubsystem.usePIDF) {
                extendoSubsystem.usePIDF = true
            }
            xPressed = myOpMode.gamepad2.cross
            if (myOpMode.gamepad2.left_stick_y > deadZone && extendoSubsystem.usePIDF) {
                extendoSubsystem.setPowerE(extendoSubsystem.minExtensionTicks.toDouble())
            } else if (myOpMode.gamepad2.left_stick_y < -deadZone && extendoSubsystem.usePIDF) {
                extendoSubsystem.setPowerE(extendoSubsystem.maxExtensionTicks.toDouble())
            } else {
                extendoSubsystem.stopE()
                extendoSubsystem.idleE()
            }
            if (!extendoSubsystem.usePIDF) {
                extendoSubsystem.setPowerE(
                    -myOpMode.gamepad2.left_stick_y.toDouble()
                )
                extendoSubsystem.setPowerR(
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
            clawSubsystem.flipBack()
            clawSubsystem.closeBoth()
            clawSubsystem.update()
        }
    }
}
