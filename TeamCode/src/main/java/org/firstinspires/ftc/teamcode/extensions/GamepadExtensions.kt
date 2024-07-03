package org.firstinspires.ftc.teamcode.extensions

import com.qualcomm.robotcore.hardware.Gamepad

object GamepadExtensions {
    enum class PushButtons {
        CROSS, CIRCLE, TRIANGLE, SQUARE, LEFT_BUMPER, RIGHT_BUMPER, LEFT_STICK_BUTTON, RIGHT_STICK_BUTTON, DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT
    }

    private var previousCross = false
    private var previousCircle = false
    private var previousTriangle = false
    private var previousSquare = false
    private var previousL1 = false
    private var previousR1 = false
    private var previousL3 = false
    private var previousR3 = false
    private var previousDpadUp = false
    private var previousDpadDown = false
    private var previousDpadLeft = false
    private var previousDpadRight = false

    private var previousCross2 = false
    private var previousCircle2 = false
    private var previousTriangle2 = false
    private var previousSquare2 = false
    private var previousL12 = false
    private var previousR12 = false
    private var previousL32 = false
    private var previousR32 = false
    private var previousDpadUp2 = false
    private var previousDpadDown2 = false
    private var previousDpadLeft2 = false
    private var previousDpadRight2 = false

    fun Gamepad.buttonJustReleased(button: PushButtons, controller: Int): Boolean {
        when (controller) {
            1 ->
                return when (button) {
                    PushButtons.CROSS -> previousCross && !this.cross
                    PushButtons.CIRCLE -> previousCircle && !this.circle
                    PushButtons.TRIANGLE -> previousTriangle && !this.triangle
                    PushButtons.SQUARE -> previousSquare && !this.square
                    PushButtons.LEFT_BUMPER -> previousL1 && !this.left_bumper
                    PushButtons.RIGHT_BUMPER -> previousR1 && !this.right_bumper
                    PushButtons.LEFT_STICK_BUTTON -> previousL3 && !this.left_stick_button
                    PushButtons.RIGHT_STICK_BUTTON -> previousR3 && !this.right_stick_button
                    PushButtons.DPAD_UP -> previousDpadUp && !this.dpad_up
                    PushButtons.DPAD_DOWN -> previousDpadDown && !this.dpad_down
                    PushButtons.DPAD_LEFT -> previousDpadLeft && !this.dpad_left
                    PushButtons.DPAD_RIGHT -> previousDpadRight && !this.dpad_right
                }

            2 ->
                return when (button) {
                    PushButtons.CROSS -> previousCross2 && !this.cross
                    PushButtons.CIRCLE -> previousCircle2 && !this.circle
                    PushButtons.TRIANGLE -> previousTriangle2 && !this.triangle
                    PushButtons.SQUARE -> previousSquare2 && !this.square
                    PushButtons.LEFT_BUMPER -> previousL12 && !this.left_bumper
                    PushButtons.RIGHT_BUMPER -> previousR12 && !this.right_bumper
                    PushButtons.LEFT_STICK_BUTTON -> previousL32 && !this.left_stick_button
                    PushButtons.RIGHT_STICK_BUTTON -> previousR32 && !this.right_stick_button
                    PushButtons.DPAD_UP -> previousDpadUp2 && !this.dpad_up
                    PushButtons.DPAD_DOWN -> previousDpadDown2 && !this.dpad_down
                    PushButtons.DPAD_LEFT -> previousDpadLeft2 && !this.dpad_left
                    PushButtons.DPAD_RIGHT -> previousDpadRight2 && !this.dpad_right
                }

        }
        return false
    }

    fun Gamepad.update(controller: Int) {
        when (controller) {
            1 -> {
                previousCross = this.cross
                previousCircle = this.circle
                previousTriangle = this.triangle
                previousSquare = this.square
                previousL1 = this.left_bumper
                previousR1 = this.right_bumper
                previousL3 = this.left_stick_button
                previousR3 = this.right_stick_button
                previousDpadUp = this.dpad_up
                previousDpadDown = this.dpad_down
                previousDpadLeft = this.dpad_left
                previousDpadRight = this.dpad_right
            }

            2 -> {
                previousCross2 = this.cross
                previousCircle2 = this.circle
                previousTriangle2 = this.triangle
                previousSquare2 = this.square
                previousL12 = this.left_bumper
                previousR12 = this.right_bumper
                previousL32 = this.left_stick_button
                previousR32 = this.right_stick_button
                previousDpadUp2 = this.dpad_up
                previousDpadDown2 = this.dpad_down
                previousDpadLeft2 = this.dpad_left
                previousDpadRight2 = this.dpad_right
            }
        }
    }

    fun Gamepad.buttonJustPressed(button: PushButtons, controller: Int): Boolean {
        when (controller) {
            1 ->
                return when (button) {
                    PushButtons.CROSS -> !previousCross && this.cross
                    PushButtons.CIRCLE -> !previousCircle && this.circle
                    PushButtons.TRIANGLE -> !previousTriangle && this.triangle
                    PushButtons.SQUARE -> !previousSquare && this.square
                    PushButtons.LEFT_BUMPER -> !previousL1 && this.left_bumper
                    PushButtons.RIGHT_BUMPER -> !previousR1 && this.right_bumper
                    PushButtons.LEFT_STICK_BUTTON -> !previousL3 && this.left_stick_button
                    PushButtons.RIGHT_STICK_BUTTON -> !previousR3 && this.right_stick_button
                    PushButtons.DPAD_UP -> !previousDpadUp && this.dpad_up
                    PushButtons.DPAD_DOWN -> !previousDpadDown && this.dpad_down
                    PushButtons.DPAD_LEFT -> !previousDpadLeft && this.dpad_left
                    PushButtons.DPAD_RIGHT -> !previousDpadRight && this.dpad_right
                }

            2 ->
                return when (button) {
                    PushButtons.CROSS -> !previousCross2 && this.cross
                    PushButtons.CIRCLE -> !previousCircle2 && this.circle
                    PushButtons.TRIANGLE -> !previousTriangle2 && this.triangle
                    PushButtons.SQUARE -> !previousSquare2 && this.square
                    PushButtons.LEFT_BUMPER -> !previousL12 && this.left_bumper
                    PushButtons.RIGHT_BUMPER -> !previousR12 && this.right_bumper
                    PushButtons.LEFT_STICK_BUTTON -> !previousL32 && this.left_stick_button
                    PushButtons.RIGHT_STICK_BUTTON -> !previousR32 && this.right_stick_button
                    PushButtons.DPAD_UP -> !previousDpadUp2 && this.dpad_up
                    PushButtons.DPAD_DOWN -> !previousDpadDown2 && this.dpad_down
                    PushButtons.DPAD_LEFT -> !previousDpadLeft2 && this.dpad_left
                    PushButtons.DPAD_RIGHT -> !previousDpadRight2 && this.dpad_right
                }
        }
        return false
    }
}