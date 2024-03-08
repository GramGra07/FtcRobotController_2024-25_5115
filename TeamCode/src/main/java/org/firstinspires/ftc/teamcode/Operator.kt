package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.LoopTime
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.ExtendoSubsystem

object Operator {
    private var touchPressed = false
    private var xPressed = false
    fun bindOtherButtons(
        myOpMode: OpMode,
        clawSubsystem: ClawSubsystem,
        extendoSubsystem: ExtendoSubsystem,
        driveSubsystem: DriveSubsystem,
    ) {
        // "Camden", "Grady", "Michael","Graden", "Delaney", "Child"
//        if (!airplaneArmed && timer.seconds() > 80) {
//            airplaneArmed = true;
//            myOpMode.gamepad2.runRumbleEffect(cRE);
//        }
//        Blink.selectLights(myOpMode);
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
                clawSubsystem.closeLeft()
//                ServoUtil.closeClaw(HardwareConfig.claw1)
            }
            if (myOpMode.gamepad2.left_bumper) {
                clawSubsystem.closeRight()
//                ServoUtil.closeClaw(HardwareConfig.claw2)
            }
            if (myOpMode.gamepad2.right_trigger > 0) {
                clawSubsystem.openLeft()
//                ServoUtil.openClaw(HardwareConfig.claw1)
            }
            if (myOpMode.gamepad2.left_trigger > 0) {
                clawSubsystem.openRight()
//                ServoUtil.openClaw(HardwareConfig.claw2)
            }

            //
            if (myOpMode.gamepad2.dpad_left) {
                clawSubsystem.flipBack()
//                flipServo.calcFlipPose(ServoUtil.backClaw.toDouble())
            } else if (myOpMode.gamepad2.dpad_up) {
                clawSubsystem.flipHigh()
//                flipServo.calcFlipPose(70.0)
            } else if (myOpMode.gamepad2.dpad_down) {
                clawSubsystem.flipZero()
//                flipServo.calcFlipPose(0.0)
            }
//            if (PastAngle.pastAngleVal != potentiometer.potentAngle()) {
//                flipServo.calcFlipPose(lastSetVal.toDouble())
//            }
            //
            if (myOpMode.gamepad2.right_stick_y < -driveSubsystem.deadZone && extendoSubsystem.usePIDF) {
                extendoSubsystem.setPowerR(extendoSubsystem.maxRotationTicks.toDouble())
//                HardwareConfig.rotationPower = Range.clip(
//                    HardwareConfig.rotationPIDF.calculate(
//                        HardwareConfig.motorRotation.currentPosition.toDouble(),
//                        Limits.maxRotationTicks.toDouble()
//                    ), -1.0, 1.0
//                )
            } else if (myOpMode.gamepad2.right_stick_y > driveSubsystem.deadZone && extendoSubsystem.usePIDF) {
                extendoSubsystem.setPowerR(extendoSubsystem.minRotationTicks.toDouble())
//                HardwareConfig.rotationPIDF.setPIDF(
//                    PIDVals.rotationPIDFCo.p,
//                    PIDVals.rotationPIDFCo.i,
//                    PIDVals.rotationPIDFCo.d,
//                    PIDVals.rotationPIDFCo.f
//                )
//                HardwareConfig.rotationPower = Range.clip(
//                    HardwareConfig.rotationPIDF.calculate(
//                        HardwareConfig.motorRotation.currentPosition.toDouble(),
//                        Limits.minRotationTicks.toDouble()
//                    ), -1.0, 1.0
//                )
            } else {
                extendoSubsystem.stopR()
//                HardwareConfig.rotationPower = 0.0
            }
            //            rotationPower = Range.clip(-myOpMode.gamepad2.right_stick_y, flipperMin, flipperMax);
            if (myOpMode.gamepad2.cross && !xPressed && extendoSubsystem.usePIDF) {
                extendoSubsystem.usePIDF = false
            } else if (myOpMode.gamepad2.cross && !xPressed && !extendoSubsystem.usePIDF) {
                extendoSubsystem.usePIDF = true
            }
            xPressed = myOpMode.gamepad2.cross
            if (myOpMode.gamepad2.left_stick_y > driveSubsystem.deadZone && extendoSubsystem.usePIDF) {
//                HardwareConfig.extensionPIDF.setPIDF(
//                    PIDVals.extensionPIDFCo.p,
//                    PIDVals.extensionPIDFCo.i,
//                    PIDVals.extensionPIDFCo.d,
//                    PIDVals.extensionPIDFCo.f
//                ) // allows to use dashboard
//                extensionPower = Range.clip(
//                    HardwareConfig.extensionPIDF.calculate(
//                        HardwareConfig.motorExtension.currentPosition.toDouble(),
//                        Limits.minExtensionTicks.toDouble()
//                    ), -1.0, 1.0
//                )
//                extendoSubsystem.setPowerE(Range.clip(
//                    extendoSubsystem.extensionPIDF.calculate(
//                        extendoSubsystem.motorExtension!!.currentPosition.toDouble(),
//                        Limits.minExtensionTicks.toDouble()
//                    ), -1.0, 1.0
//                ))
                extendoSubsystem.setPowerE(extendoSubsystem.minExtensionTicks.toDouble())
            } else if (myOpMode.gamepad2.left_stick_y < -driveSubsystem.deadZone && extendoSubsystem.usePIDF) {
                extendoSubsystem.setPowerE(extendoSubsystem.maxExtensionTicks.toDouble())
//                HardwareConfig.extensionPIDF.setPIDF(
//                    PIDVals.extensionPIDFCo.p,
//                    PIDVals.extensionPIDFCo.i,
//                    PIDVals.extensionPIDFCo.d,
//                    PIDVals.extensionPIDFCo.f
//                ) // allows to use dashboard
//                extensionPower = Range.clip(
//                    HardwareConfig.extensionPIDF.calculate(
//                        HardwareConfig.motorExtension.currentPosition.toDouble(),
//                        Limits.maxExtensionTicks.toDouble()
//                    ), -1.0, 1.0
//                )
            } else {
//                extensionPower = 0.0
                extendoSubsystem.stopE()
            }
            if (!extendoSubsystem.usePIDF) {
                extendoSubsystem.setPowerE(
                    -myOpMode.gamepad2.left_stick_y.toDouble()
                )
                extendoSubsystem.setPowerR(
                    -myOpMode.gamepad2.right_stick_y.toDouble()
                )
//                HardwareConfig.extensionPower = Range.clip(
//                    -myOpMode.gamepad2.left_stick_y.toDouble(),
//                    Limits.slideMin,
//                    Limits.slideMax
//                )
//                HardwareConfig.rotationPower = Range.clip(
//                    -myOpMode.gamepad2.right_stick_y.toDouble(),
//                    Limits.flipperMin,
//                    Limits.flipperMax
//                )
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
