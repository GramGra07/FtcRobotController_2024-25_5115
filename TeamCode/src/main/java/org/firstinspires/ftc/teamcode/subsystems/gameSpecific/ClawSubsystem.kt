package org.firstinspires.ftc.teamcode.subsystems.gameSpecific

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.calcFlipPose
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.initServo
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.ServoUtil


class ClawSubsystem(ahwMap: HardwareMap) {
    enum class ClawStates {
        OPEN,
        CLOSED,
        IDLE
    }

    enum class FlipStates {
        HIGH,
        BACK,
        ZERO,
        IDLE
    }

    private var claw1: Servo
    private var claw2: Servo
    private var flipServo: Servo

    init {
        claw1 = initServo(ahwMap, "claw1")
        claw2 = initServo(ahwMap, "claw2")
        flipServo = initServo(ahwMap, "flipServo")
        closeBoth()
        flipBack()
        update()
    }

    class clawDefault : CommandBase() {
        private fun update() {
            this.update()
        }

        override fun execute() {
            update()
        }
    }

    private var rightState = ClawStates.CLOSED
    private var leftState = ClawStates.CLOSED
    private var flipState = FlipStates.BACK

    private fun openBoth() {
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

    fun flipBack() {
        flipState = FlipStates.BACK
    }

    fun flipZero() {
        flipState = FlipStates.ZERO
    }

    fun update() {
        when (rightState) {
            ClawStates.OPEN -> {
                ServoUtil.openClaw1(claw1)
                rightState = ClawStates.IDLE
            }

            ClawStates.CLOSED -> {
                ServoUtil.closeClaw1(claw1)
                rightState = ClawStates.IDLE
            }

            ClawStates.IDLE -> {}
        }
        when (leftState) {
            ClawStates.OPEN -> {
                ServoUtil.openClaw2(claw2)
                leftState = ClawStates.IDLE
            }

            ClawStates.CLOSED -> {
                ServoUtil.closeClaw2(claw2)
                leftState = ClawStates.IDLE
            }

            ClawStates.IDLE -> {}
        }
        when (flipState) {
            FlipStates.HIGH -> {
                flipServo.calcFlipPose(70.0)
                flipState = FlipStates.IDLE
            }

            FlipStates.BACK -> {
                flipServo.calcFlipPose(ServoUtil.backClaw.toDouble())
                flipState = FlipStates.IDLE
            }

            FlipStates.ZERO -> {
                flipServo.calcFlipPose(0.0)
                flipState = FlipStates.IDLE
            }

            FlipStates.IDLE -> {}
        }
//        if (PotentPositions.pastAngleVal != potentiometer.potentAngle()) {
//            flipServo.calcFlipPose(ServoExtensions.lastSetVal.toDouble())
//        }
    }
}