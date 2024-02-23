package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.DeadZones
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.LoopTime
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.PIDVals
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.PastAngle
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.varConfig
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.potentAngle
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.calcFlipPose
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.lastSetVal
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.extensionPower
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.flipServo
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.potentiometer
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive

object Operator {
    private var touchPressed = false
    private var xPressed = false
    fun bindOtherButtons(myOpMode: OpMode, drive: MecanumDrive) {
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
                ServoUtil.closeClaw(HardwareConfig.claw1)
            }
            if (myOpMode.gamepad2.left_bumper) {
                ServoUtil.closeClaw(HardwareConfig.claw2)
            }
            if (myOpMode.gamepad2.right_trigger > 0) {
                ServoUtil.openClaw(HardwareConfig.claw1)
            }
            if (myOpMode.gamepad2.left_trigger > 0) {
                ServoUtil.openClaw(HardwareConfig.claw2)
            }
            //
            if (myOpMode.gamepad2.dpad_left) {
                flipServo.calcFlipPose(ServoUtil.backClaw.toDouble())
            } else if (myOpMode.gamepad2.dpad_up) {
                flipServo.calcFlipPose(70.0)
            } else if (myOpMode.gamepad2.dpad_down) {
                flipServo.calcFlipPose(0.0)
            }
            if (PastAngle.pastAngleVal != potentiometer.potentAngle()) {
                flipServo.calcFlipPose(lastSetVal.toDouble())
            }
            //
            if (myOpMode.gamepad2.right_stick_y < -DeadZones.deadZone && varConfig.usePIDF) {
                HardwareConfig.rotationPIDF.setPIDF(
                    PIDVals.rotationPIDFCo.p,
                    PIDVals.rotationPIDFCo.i,
                    PIDVals.rotationPIDFCo.d,
                    PIDVals.rotationPIDFCo.f
                )
                HardwareConfig.rotationPower = Range.clip(
                    HardwareConfig.rotationPIDF.calculate(
                        HardwareConfig.motorRotation.currentPosition.toDouble(),
                        Limits.maxRotationTicks.toDouble()
                    ), -1.0, 1.0
                )
            } else if (myOpMode.gamepad2.right_stick_y > DeadZones.deadZone && varConfig.usePIDF) {
                HardwareConfig.rotationPIDF.setPIDF(
                    PIDVals.rotationPIDFCo.p,
                    PIDVals.rotationPIDFCo.i,
                    PIDVals.rotationPIDFCo.d,
                    PIDVals.rotationPIDFCo.f
                )
                HardwareConfig.rotationPower = Range.clip(
                    HardwareConfig.rotationPIDF.calculate(
                        HardwareConfig.motorRotation.currentPosition.toDouble(),
                        Limits.minRotationTicks.toDouble()
                    ), -1.0, 1.0
                )
            } else {
                HardwareConfig.rotationPower = 0.0
            }
            //            rotationPower = Range.clip(-myOpMode.gamepad2.right_stick_y, flipperMin, flipperMax);
            if (myOpMode.gamepad2.cross && !xPressed && varConfig.usePIDF) {
                varConfig.usePIDF = false
            } else if (myOpMode.gamepad2.cross && !xPressed && !varConfig.usePIDF) {
                varConfig.usePIDF = true
            }
            xPressed = myOpMode.gamepad2.cross
            if (myOpMode.gamepad2.left_stick_y > DeadZones.deadZone && varConfig.usePIDF) {
                HardwareConfig.extensionPIDF.setPIDF(
                    PIDVals.extensionPIDFCo.p,
                    PIDVals.extensionPIDFCo.i,
                    PIDVals.extensionPIDFCo.d,
                    PIDVals.extensionPIDFCo.f
                ) // allows to use dashboard
                HardwareConfig.extensionPower = Range.clip(
                    HardwareConfig.extensionPIDF.calculate(
                        HardwareConfig.motorExtension.currentPosition.toDouble(),
                        Limits.minExtensionTicks.toDouble()
                    ), -1.0, 1.0
                )
            } else if (myOpMode.gamepad2.left_stick_y < -DeadZones.deadZone && varConfig.usePIDF) {
                HardwareConfig.extensionPIDF.setPIDF(
                    PIDVals.extensionPIDFCo.p,
                    PIDVals.extensionPIDFCo.i,
                    PIDVals.extensionPIDFCo.d,
                    PIDVals.extensionPIDFCo.f
                ) // allows to use dashboard
                HardwareConfig.extensionPower = Range.clip(
                    HardwareConfig.extensionPIDF.calculate(
                        HardwareConfig.motorExtension.currentPosition.toDouble(),
                        Limits.maxExtensionTicks.toDouble()
                    ), -1.0, 1.0
                )
            }else{
                extensionPower = 0.0
            }
            if (!varConfig.usePIDF) {
                HardwareConfig.extensionPower = Range.clip(
                    -myOpMode.gamepad2.left_stick_y.toDouble(),
                    Limits.slideMin,
                    Limits.slideMax
                )
                HardwareConfig.rotationPower = Range.clip(
                    -myOpMode.gamepad2.right_stick_y.toDouble(),
                    Limits.flipperMin,
                    Limits.flipperMax
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
            flipServo.calcFlipPose(30.0)
        }
    }
}
