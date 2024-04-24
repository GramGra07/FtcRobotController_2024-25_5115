package org.firstinspires.ftc.teamcode.subsystems.humanInput

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.LoopTime
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.ClawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.ExtendoSubsystem

@Config
object Operator {

    @JvmField
    var deadZone = 0.15

    private var touchPressed = false
    private var xPressed = false
    fun bindOtherButtons(
        myOpMode: OpMode,
        clawSubsystem: ClawSubsystem,
        extendoSubsystem: ExtendoSubsystem,
        driveSubsystem: DriveSubsystem,
    ) {
        // "Camden", "Grady", "Michael","Graden", "Delaney", "Child"
        val otherControls = Drivers.otherControls
        val currOther = Drivers.currOther
        if (currOther === otherControls[0]) { //Camden
            if (!touchPressed && myOpMode.gamepad2.touchpad && LoopTime.useLoopTime) {
                LoopTime.useLoopTime = false
            } else if (!touchPressed && myOpMode.gamepad2.touchpad && !LoopTime.useLoopTime) {
                LoopTime.useLoopTime = true
            }
            touchPressed = myOpMode.gamepad2.touchpad
            if (myOpMode.gamepad2.right_bumper) {
//                clawSubsystem.closeLeft()
                runBlocking(clawSubsystem.closeLeftAction())
//                ServoUtil.closeClaw(HardwareConfig.claw1)
            }
            if (myOpMode.gamepad2.left_bumper) {
                runBlocking(clawSubsystem.closeRightAction())
//                clawSubsystem.closeRight()
//                ServoUtil.closeClaw(HardwareConfig.claw2)
            }
            if (myOpMode.gamepad2.right_trigger > 0) {
//                clawSubsystem.openLeft()
                runBlocking(clawSubsystem.openLeftAction())
//                ServoUtil.openClaw(HardwareConfig.claw1)
            }
            if (myOpMode.gamepad2.left_trigger > 0) {
                clawSubsystem.openRightAction()
                runBlocking(clawSubsystem.openRightAction())
//                ServoUtil.openClaw(HardwareConfig.claw2)
            }

            //
            if (myOpMode.gamepad2.dpad_left) {
//                clawSubsystem.flipBack()
                runBlocking(clawSubsystem.flipBackAction())
            } else if (myOpMode.gamepad2.dpad_up) {
//                clawSubsystem.flipHigh()
                runBlocking(clawSubsystem.flipHighAction())
            } else if (myOpMode.gamepad2.dpad_down) {
//                clawSubsystem.flipZero()
                runBlocking(clawSubsystem.flipZeroAction())
            }
            if (myOpMode.gamepad2.right_stick_y < -deadZone && extendoSubsystem.usePIDF) {
                extendoSubsystem.setPowerR(extendoSubsystem.maxRotationTicks.toDouble())
            } else if (myOpMode.gamepad2.right_stick_y > deadZone && extendoSubsystem.usePIDF) {
                extendoSubsystem.setPowerR(extendoSubsystem.minRotationTicks.toDouble())
            } else {
                extendoSubsystem.stopR()
                extendoSubsystem.idleR()
//                HardwareConfig.rotationPower = 0.0
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
//                extensionPower = 0.0
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
//            clawSubsystem.flipBack()
            runBlocking(clawSubsystem.flipBackAction())
            runBlocking(clawSubsystem.closeBothAction())
//            clawSubsystem.closeBoth()
            clawSubsystem.update()
        }
    }
}
