package org.firstinspires.ftc.teamcode.utilClass

import com.acmerobotics.roadrunner.Pose2d
import kotlin.math.abs

class MathFunctions {
//    var position = 0.0 //sets servo position to 0-1 multiplier
//    val degree_mult = 0.00555555554 // = 100/180
//    fun calcServo(degrees: Int): Double {
//        position = degree_mult * degrees
//        return position
//    }

    fun averageOf(values: DoubleArray): Double {
        return values.average()
    }

    fun getQuadrant(pose: Pose2d): Int {
        var x = pose.position.x.toInt()
        var y = pose.position.y.toInt()
        if (x == 0) {
            x = (x + 0.1).toInt() // compensates for divide by 0 error
        }
        if (y == 0) {
            y = (y + 0.1).toInt() // compensates for divide by 0 error
        }
        val xSign = x.toDouble() / abs(x) // gets sign of x
        val ySign = y.toDouble() / abs(y) // gets sign of y
        val xIsPositive = xSign == 1.0 // checks if x is positive
        val yIsPositive = ySign == 1.0 // checks if y is positive
        return if (xIsPositive && !yIsPositive) {
            1
        } else if (xIsPositive && yIsPositive) {
            2
        } else if (!xIsPositive && !yIsPositive) {
            3
        } else if (!xIsPositive && yIsPositive) {
            4
        } else {
            0
        }
    }

    fun threeFourths(amount: Int): Int {
        return amount / 4 * 3
    }

    companion object {
        fun normDelta(delta: Double): Double {
            var result = delta
            while (result >= 180.0) result -= 360.0
            while (result < -180.0) result += 360.0
            return result
        }

        fun inRange(value: Double, min: Double, max: Double): Boolean {
            return value in min..max
        }

        fun inTolerance(value: Double, value2: Double, tolerance: Double): Boolean {
            return value in value2 - tolerance..value2 + tolerance
        }
    }
}
