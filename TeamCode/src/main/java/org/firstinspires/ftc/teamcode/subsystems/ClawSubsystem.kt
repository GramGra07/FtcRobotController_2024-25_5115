package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.closeClaw
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.openClaw
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.AutoServoPositions
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.PastAngle
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.potentAngle
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.calcFlipPose
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.initServo
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.potentiometer

class ClawSubsystem(ahwMap: HardwareMap) {
    enum class ClawStates {
        OPEN,
        CLOSED
    }

    enum class FlipStates {
        HIGH,
        BACK,
        ZERO,
        UP,
        DOWN
    }

    var claw1: Servo? = null
    var claw2: Servo? = null
    private var flipServo: Servo? = null

    init {
        if (claw1 == null) {
            claw1 = initServo(ahwMap, "claw1")
        }
        if (claw2 == null) {
            claw2 = initServo(ahwMap, "claw2")
        }
        if (flipServo == null) {
            flipServo = initServo(ahwMap, "flipServo")
        }
        closeBoth()
        update()
    }

    private var states: Array<ClawStates> = arrayOf(ClawStates.OPEN, ClawStates.OPEN)
    private var flipState = FlipStates.BACK
    fun openBoth() {
        states[0] = ClawStates.OPEN
        states[1] = ClawStates.OPEN
    }

    fun openLeft() {
        states[0] = ClawStates.OPEN
    }

    fun openRight() {
        states[1] = ClawStates.OPEN
    }

    fun closeBoth() {
        states[0] = ClawStates.CLOSED
        states[1] = ClawStates.CLOSED
    }

    fun closeLeft() {
        states[0] = ClawStates.CLOSED
    }

    fun closeRight() {
        states[1] = ClawStates.CLOSED
    }

    fun flipHigh() {
        flipState = FlipStates.HIGH
    }

    fun flipUp() {
        flipState = FlipStates.UP
    }

    fun flipBack() {
        flipState = FlipStates.BACK
    }

    fun flipDown() {
        flipState = FlipStates.DOWN
    }

    fun flipZero() {
        flipState = FlipStates.ZERO
    }

    fun update() {
        when (states[0]) {
            ClawStates.OPEN -> openClaw(claw1!!)
            ClawStates.CLOSED -> closeClaw(claw1!!)
        }
        when (states[1]) {
            ClawStates.OPEN -> openClaw(claw2!!)
            ClawStates.CLOSED -> closeClaw(claw2!!)
        }
        when (flipState) {
            FlipStates.HIGH -> flipServo!!.calcFlipPose(70.0)
            FlipStates.BACK -> flipServo!!.calcFlipPose(ServoUtil.backClaw.toDouble())
            FlipStates.ZERO -> flipServo!!.calcFlipPose(0.0)
            FlipStates.UP -> flipServo!!.calcFlipPose(AutoServoPositions.flipUp.toDouble())
            FlipStates.DOWN -> flipServo!!.calcFlipPose((AutoServoPositions.flipDown - 10).toDouble())
        }
        if (PastAngle.pastAngleVal != potentiometer.potentAngle()) {
            flipServo!!.calcFlipPose(ServoExtensions.lastSetVal.toDouble())
        }
    }
}