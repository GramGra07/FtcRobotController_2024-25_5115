import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

class HummingbirdDrive {

    // Constants for wheel parameters
    val WHEEL_DISTANCE = 12.0 // distance between wheels in inches
    val WHEEL_RADIUS = 2.0 // radius of each wheel in inches
    val TICKS_PER_REVOLUTION = 1440.0 // encoder ticks per wheel revolution

    // Constants for PID control
    val KP_HEADING = 0.1 // Proportional gain for heading control
    val KI_HEADING = 0.01 // Integral gain for heading control
    val KD_HEADING = 0.05 // Derivative gain for heading control
    val KP_DISTANCE = 0.1 // Proportional gain for distance control
    val TARGET_TOLERANCE = 1.0 // Tolerance for target heading

    // Variables for robot position and orientation
    var robotX = 0.0 // in inches
    var robotY = 0.0 // in inches
    var robotHeading = 0.0 // in degrees

    // Variables for PID control
    var headingIntegralError = 0.0
    var headingPreviousError = 0.0
    var distanceIntegralError = 0.0
    var distancePreviousError = 0.0

    // Function to update robot position and orientation based on encoder readings
    fun updatePosition(leftTicks: Int, rightTicks: Int, rearTicks: Int) {
        // Calculate distance traveled by each wheel
        val leftDistance = (leftTicks / TICKS_PER_REVOLUTION) * (PI * WHEEL_RADIUS)
        val rightDistance = (rightTicks / TICKS_PER_REVOLUTION) * (PI * WHEEL_RADIUS)
        val rearDistance = (rearTicks / TICKS_PER_REVOLUTION) * (PI * WHEEL_RADIUS)

        // Calculate robot's movement and rotation
        val distance = (leftDistance + rightDistance) / 2.0
        val rotation = (rightDistance - leftDistance) / WHEEL_DISTANCE

        // Update robot position and orientation
        robotX += distance * cos(Math.toRadians(robotHeading + rotation / 2.0))
        robotY += distance * sin(Math.toRadians(robotHeading + rotation / 2.0))
        robotHeading += Math.toDegrees(rotation) + Math.toDegrees(rearDistance / WHEEL_DISTANCE)
    }

    // Function to calculate PID control output for heading control
    fun calculateHeadingPIDError(currentHeading: Double, targetHeading: Double): Double {
        val error = targetHeading - currentHeading
        headingIntegralError += error
        val derivativeError = error - headingPreviousError
        headingPreviousError = error
        return KP_HEADING * error + KI_HEADING * headingIntegralError + KD_HEADING * derivativeError
    }

    // Function to calculate PID control output for distance control
    fun calculateDistancePIDError(currentDistance: Double, targetDistance: Double): Double {
        val error = targetDistance - currentDistance
        distanceIntegralError += error
        val derivativeError = error - distancePreviousError
        distancePreviousError = error
        return KP_DISTANCE * error + KI_HEADING * distanceIntegralError + KD_HEADING * derivativeError
    }

    // Function to adjust individual wheel speeds based on PID control outputs
    fun adjustWheelSpeeds(headingPIDOutput: Double, distancePIDOutput: Double): List<Double> {
        // Calculate wheel speeds based on PID outputs (implement your own logic)
        // For simplicity, let's assume the outputs directly control wheel speeds
        val wheelSpeeds = mutableListOf<Double>()
        wheelSpeeds.add(headingPIDOutput + distancePIDOutput) // Front left wheel
        wheelSpeeds.add(-headingPIDOutput + distancePIDOutput) // Front right wheel
        wheelSpeeds.add(headingPIDOutput + distancePIDOutput) // Rear wheel
        return wheelSpeeds
    }

    // Function to follow a path defined by Bezier splines
    fun followPathUsingBezierSplines(controlPoints: List<Pair<Double, Double>>, numSegments: Int) {
        for (i in 0 until numSegments) {
            // Calculate t value for the current segment
            val t0 = i.toDouble() / numSegments
            val t1 = (i + 1).toDouble() / numSegments

            // Calculate control points for the current segment
            val p0 = calculateBezierPoint(controlPoints, t0)
            val p1 = calculateBezierPoint(controlPoints, t1)

            // Calculate heading and distance errors
            val dx = p1.first - robotX
            val dy = p1.second - robotY
            val targetHeading = Math.toDegrees(atan2(dy, dx))
            val distanceError = sqrt(dx * dx + dy * dy)

            // Calculate PID control outputs for heading and distance control
            val headingPIDOutput = calculateHeadingPIDError(robotHeading, targetHeading)
            val distancePIDOutput = calculateDistancePIDError(0.0, distanceError)

            // Adjust individual wheel speeds based on PID control outputs
            val wheelSpeeds = adjustWheelSpeeds(headingPIDOutput, distancePIDOutput)

            // Output wheel speeds (for demonstration purposes)
            println("Wheel Speeds:")
            for ((index, speed) in wheelSpeeds.withIndex()) {
                println("Wheel ${index + 1}: $speed")
            }

            // Simulate robot movement (for demonstration purposes)
            // Implement your own code to actually control the robot based on wheel speeds
            updatePosition(100, 110, 120)
        }
    }

    // Function to calculate a point on the Bezier curve given control points and parameter t
    fun calculateBezierPoint(
        controlPoints: List<Pair<Double, Double>>,
        t: Double
    ): Pair<Double, Double> {
        val n = controlPoints.size - 1
        var x = 0.0
        var y = 0.0
        for (i in 0..n) {
            val i2 = i.toDouble()
            val coefficient = binomialCoefficient(n, i) * Math.pow(1 - t, n - i2) * Math.pow(t, i2)
            x += coefficient * controlPoints[i].first
            y += coefficient * controlPoints[i].second
        }
        return Pair(x, y)
    }

    // Function to calculate binomial coefficient
    fun binomialCoefficient(n: Int, k: Int): Int {
        var result = 1
        for (i in 1..k) {
            result *= (n - i + 1) / i
        }
        return result
    }

    // Example usage
    fun main() {
        // Define control points for the Bezier spline path
        val controlPoints = listOf(
            Pair(0.0, 0.0), // Example control points (x, y)
            Pair(12.0, 12.0),
            Pair(24.0, 12.0),
            Pair(36.0, 0.0)
        )

        // Follow the defined path using Bezier splines
        followPathUsingBezierSplines(controlPoints, numSegments = 100)
    }

}