package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d

class MathFunctions {
    var position = 0.0 //sets servo position to 0-1 multiplier
    val degree_mult = 0.00555555554 // = 100/180
    fun calcServo(degrees: Int): Double {
        position = degree_mult * degrees
        return position
    }

    fun averageOf(values: DoubleArray): Double {
        var sum = 0.0
        for (value in values) {
            sum += value
        }
        return sum / values.size
    }

    companion object {
        fun inRange(value: Double, min: Double, max: Double): Boolean {
            return value >= min && value <= max
        }

        fun getQuadrant(pose: Pose2d): Int {
            var x = pose.x.toInt()
            var y = pose.y.toInt()
            if (x == 0) {
                x = (x + 0.1).toInt() // compensates for divide by 0 error
            }
            if (y == 0) {
                y = (y + 0.1).toInt() // compensates for divide by 0 error
            }
            val xSign = x.toDouble() / Math.abs(x) // gets sign of x
            val ySign = y.toDouble() / Math.abs(y) // gets sign of y
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
    }
}
