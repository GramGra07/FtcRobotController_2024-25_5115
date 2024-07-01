package org.firstinspires.ftc.teamcode.customHardware.gamepad

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.utilClass.objects.Point
import org.opencv.core.Rect

class CustomGamepad(private var gamepad: Gamepad) {
    private var crossHeld: Boolean = false
    private var circleHeld: Boolean = false
    private var squareHeld: Boolean = false
    private var triangleHeld: Boolean = false
    private var dpadUpHeld: Boolean = false
    private var dpadDownHeld: Boolean = false
    private var dpadLeftHeld: Boolean = false
    private var dpadRightHeld: Boolean = false
    private var leftStickButtonHeld: Boolean = false
    private var rightStickButtonHeld: Boolean = false
    private var leftBumperHeld: Boolean = false
    private var rightBumperHeld: Boolean = false
    private var touchpadHeld: Boolean = false
    private var optionsHeld: Boolean = false
    private var shareHeld: Boolean = false

    fun update() {
        crossHeld = gamepad.cross
        circleHeld = gamepad.circle
        squareHeld = gamepad.square
        triangleHeld = gamepad.triangle
        dpadUpHeld = gamepad.dpad_up
        dpadDownHeld = gamepad.dpad_down
        dpadLeftHeld = gamepad.dpad_left
        dpadRightHeld = gamepad.dpad_right
        leftStickButtonHeld = gamepad.left_stick_button
        rightStickButtonHeld = gamepad.right_stick_button
        leftBumperHeld = gamepad.left_bumper
        rightBumperHeld = gamepad.right_bumper
        touchpadHeld = gamepad.touchpad
        optionsHeld = gamepad.options
        shareHeld = gamepad.share
    }

    fun getCurrentButton(button: Button): Any {
        when (button) {
            Button.CROSS -> return gamepad.cross
            Button.CIRCLE -> return gamepad.circle
            Button.SQUARE -> return gamepad.square
            Button.TRIANGLE -> return gamepad.triangle
            Button.DPAD_UP -> return gamepad.dpad_up
            Button.DPAD_DOWN -> return gamepad.dpad_down
            Button.DPAD_LEFT -> return gamepad.dpad_left
            Button.DPAD_RIGHT -> return gamepad.dpad_right
            Button.LEFT_STICK_BUTTON -> return gamepad.left_stick_button
            Button.RIGHT_STICK_BUTTON -> return gamepad.right_stick_button
            Button.LEFT_BUMPER -> return gamepad.left_bumper
            Button.RIGHT_BUMPER -> return gamepad.right_bumper
            Button.TOUCHPAD -> return gamepad.touchpad
            Button.OPTIONS -> return gamepad.options
            Button.SHARE -> return gamepad.share
            Button.LEFT_TRIGGER -> return gamepad.left_trigger
            Button.RIGHT_TRIGGER -> return gamepad.right_trigger
            Button.TOUCHPAD_DRAG_X -> return gamepad.touchpad_finger_1_x
            Button.TOUCHPAD_DRAG_Y -> return gamepad.touchpad_finger_1_y
        }
    }

    fun getPastButton(button: Button): Boolean {
        when (button) {
            Button.CROSS -> return crossHeld
            Button.CIRCLE -> return circleHeld
            Button.SQUARE -> return squareHeld
            Button.TRIANGLE -> return triangleHeld
            Button.DPAD_UP -> return dpadUpHeld
            Button.DPAD_DOWN -> return dpadDownHeld
            Button.DPAD_LEFT -> return dpadLeftHeld
            Button.DPAD_RIGHT -> return dpadRightHeld
            Button.LEFT_STICK_BUTTON -> return leftStickButtonHeld
            Button.RIGHT_STICK_BUTTON -> return rightStickButtonHeld
            Button.LEFT_BUMPER -> return leftBumperHeld
            Button.RIGHT_BUMPER -> return rightBumperHeld
            Button.TOUCHPAD -> return touchpadHeld
            Button.OPTIONS -> return optionsHeld
            Button.SHARE -> return shareHeld
            else -> return false
        }

    }

    fun isPressed(button: Button): Boolean {
        val out = getCurrentButton(button)
        if (out is Boolean) {
            return out
        } else {
            throw Exception("Button called is not a boolean, " + button.name)
        }
    }

    fun isHeld(button: Button): Boolean {
        val out = getCurrentButton(button)
        if (out is Boolean) {
            return out && getPastButton(button)
        } else {
            throw Exception("Button called is not a boolean, " + button.name)
        }
    }

    fun justPressed(button: Button): Boolean {
        val out = getCurrentButton(button)
        if (out is Boolean) {
            return out && !getPastButton(button)
        } else {
            throw Exception("Button called is not a boolean, " + button.name)
        }
    }

    fun justReleased(button: Button): Boolean {
        val out = getCurrentButton(button)
        if (out is Boolean) {
            return !out && getPastButton(button)
        } else {
            throw Exception("Button called is not a boolean, " + button.name)
        }
    }

    fun getTrigger(button: Button): Float {
        val out = getCurrentButton(button)
        if (button == Button.LEFT_TRIGGER || button == Button.RIGHT_TRIGGER) {
            return out as Float
        } else {
            throw Exception("Button called is not a float, " + button.name)
        }
    }

    fun getTriggerOverTolerance(button: Button, tolerance: Double): Boolean {
        getTrigger(button).let {
            return it > tolerance
        }
    }

    enum class Colors {
        HOT_PINK,
        RED,
        BLACK,
        BLUE,
        GREEN
    }

    fun setColor(colors: Colors) {
        when (colors) {
            Colors.HOT_PINK -> gamepad.setLedColor(229.0, 74.0, 161.0, -1)
            Colors.RED -> gamepad.setLedColor(255.0, 0.0, 0.0, -1)
            Colors.BLACK -> gamepad.setLedColor(0.0, 0.0, 0.0, -1)
            Colors.BLUE -> gamepad.setLedColor(0.0, 0.0, 255.0, -1)
            Colors.GREEN -> gamepad.setLedColor(0.0, 255.0, 0.0, -1)
        }
    }

    fun reset() {
        crossHeld = false
        circleHeld = false
        squareHeld = false
        triangleHeld = false
        dpadUpHeld = false
        dpadDownHeld = false
        dpadLeftHeld = false
        dpadRightHeld = false
        leftStickButtonHeld = false
        rightStickButtonHeld = false
        leftBumperHeld = false
        rightBumperHeld = false
        touchpadHeld = false
        optionsHeld = false
        shareHeld = false
    }

    fun getTouchpadDrag(): Point {
        return Point(
            getCurrentButton(Button.TOUCHPAD_DRAG_X) as Double,
            getCurrentButton(Button.TOUCHPAD_DRAG_Y) as Double
        )
    }

    fun touchpadIn(rect: Rect): Boolean {
        val point = getTouchpadDrag()
        val x = point.x!!
        val y = point.y!!
        return rect.contains(org.opencv.core.Point(x, y))
    }
}