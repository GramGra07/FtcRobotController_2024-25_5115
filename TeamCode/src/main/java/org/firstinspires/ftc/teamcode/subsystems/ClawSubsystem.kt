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
        claw1 = initServo(ahwMap, "claw1")
        claw2 = initServo(ahwMap, "claw2")
        flipServo = initServo(ahwMap, "flipServo")
        closeBoth()
        update()
    }

    private var rightState = ClawStates.CLOSED
    private var leftState = ClawStates.CLOSED
    private var flipState = FlipStates.BACK
    fun openBoth() {
        rightState = ClawStates.OPEN
        leftState = ClawStates.OPEN
    }

    fun openLeft() {
        leftState = ClawStates.OPEN
    }

    fun openRight() {
        rightState = ClawStates.OPEN
    }

    fun closeBoth() {
        leftState = ClawStates.CLOSED
        rightState = ClawStates.CLOSED
    }

    fun closeLeft() {
        leftState = ClawStates.CLOSED
    }

    fun closeRight() {
        rightState = ClawStates.CLOSED
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
        when (rightState) {
            ClawStates.OPEN -> openClaw(claw1!!)
            ClawStates.CLOSED -> closeClaw(claw1!!)
        }
        when (leftState) {
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