package org.firstinspires.ftc.teamcode.subsystems.avoidance

import org.firstinspires.ftc.teamcode.Point
import org.firstinspires.ftc.teamcode.opModes.rr.drive.DriveConstants
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

class VectorField(point: Point? = Point(), radius: Double) {
    var point: Point = point!!
    private var rad = radius

    companion object {
        fun getCorrectionByAvoidance(fields: List<VectorField>, pose: Point): Map<String, Double> {
            val newPoint = getPointIfAvoid(fields, pose)
            return calculateWheelVelocities(pose.x!! - newPoint.x!!, pose.y!! - newPoint.y!!)
        }

        fun massCreate(fieldList: HashMap<Point, Double>): List<VectorField> {
            val fields = mutableListOf<VectorField>()
            for (field in fieldList) {
                fields.add(VectorField(field.key, field.value))
            }
            return fields
        }

        fun poseInField(pose: Point, field: VectorField): Boolean {
            return sqrt((pose.x!! - field.point.x!!).pow(2.0) + (pose.y!! - field.point.y!!).pow(2.0)) < field.rad
        }

        fun closestFree(pose: Point, field: VectorField): Point {
            val x = pose.x!!
            val y = pose.y!!
            val x0 = field.point.x!!
            val y0 = field.point.y!!
            val r = field.rad
            val d = sqrt((x - x0).pow(2.0) + (y - y0).pow(2.0))
            val x1 = x0 + r * (x - x0) / d
            val y1 = y0 + r * (y - y0) / d
            return Point(x1, y1)
        }

        private fun getPointIfAvoid(fields: List<VectorField>, pose: Point): Point {
            var newPose = pose
            for (field in fields) {
                if (poseInField(pose, field)) {
                    newPose = closestFree(pose, field)
                }
            }
            return newPose
        }

        private fun calculateWheelVelocities(xError: Double, yError: Double): Map<String, Double> {
            val kp = DriveConstants.MOTOR_VELO_PID.p
            val theta = atan2(yError, xError)
            val magnitude = sqrt(xError.pow(2.0) + yError.pow(2.0))
            val forwardCorrection = kp * magnitude * sin(theta)
            val strafeCorrection = kp * magnitude * cos(theta)
            val flVelocity = forwardCorrection + strafeCorrection  // Front-left wheel
            val frVelocity = forwardCorrection - strafeCorrection  // Front-right wheel
            val rlVelocity = forwardCorrection + strafeCorrection  // Rear-left wheel
            val rrVelocity = forwardCorrection - strafeCorrection  // Rear-right wheel

            return mapOf(
                "FL" to flVelocity,
                "FR" to frVelocity,
                "RL" to rlVelocity,
                "RR" to rrVelocity
            )
        }
    }
}