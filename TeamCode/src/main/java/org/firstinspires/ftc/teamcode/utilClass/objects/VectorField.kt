package org.firstinspires.ftc.teamcode.utilClass.objects

import com.acmerobotics.roadrunner.Pose2d
import org.firstinspires.ftc.teamcode.extensions.PoseExtensions.toPoint
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.varConfig
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

//@Config
class VectorField(point: Point? = Point(), radius: Double) {
    var point: Point? = point

    @JvmField
    var rad = radius

    companion object {
        private var fieldStrengthCoefficient: Double = varConfig.correctionSpeedAvoid
        private val correctionMap = mutableMapOf<String, Double>()

        fun getCorrectionByAvoidancePUSH(
            fields: List<VectorField>,
            pose: Point
        ): Map<String, Double>? {
            fieldStrengthCoefficient = varConfig.correctionSpeedAvoid
            var newPose: Point? = null
            for (field in fields) {
                if (poseInField(pose, field)) {
                    newPose = closestFree(pose, field)
                    break
                }
            }
            if (newPose == null) {
                return null
            }
            val xError = pose.x!! - newPose.x!!
            val yError = pose.y!! - newPose.y!!
            val theta = atan2(yError, xError)
            val magnitude = sqrt(xError.pow(2.0) + yError.pow(2.0))
            val forwardCorrection = fieldStrengthCoefficient * magnitude * sin(theta)
            val strafeCorrection = fieldStrengthCoefficient * magnitude * cos(theta)
            val flVelocity = forwardCorrection + strafeCorrection  // Front-left wheel
            val frVelocity = forwardCorrection - strafeCorrection * -1  // Front-right wheel
            val rlVelocity = forwardCorrection + strafeCorrection  // Rear-left wheel
            val rrVelocity = forwardCorrection - strafeCorrection * -1  // Rear-right wheel

            correctionMap["FL"] = flVelocity
            correctionMap["FR"] = frVelocity
            correctionMap["RL"] = rlVelocity
            correctionMap["RR"] = rrVelocity

            return correctionMap
        }
        fun getCorrectionByAvoidanceSTOP(
            fields: List<VectorField>,
            pose: Pose2d,
            forwardPower: Double,
            strafePower: Double,
            turnPower: Double
        ): Map<String, Double>? {
            fieldStrengthCoefficient = varConfig.correctionSpeedAvoid
            val correctionMap = mutableMapOf<String, Double>()

            val projectedPose = Pose2d(
                pose.position.x + forwardPower * cos(pose.heading.toDouble()) - strafePower * sin(pose.heading.toDouble()),
                pose.position.y + forwardPower * sin(pose.heading.toDouble()) + strafePower * cos(pose.heading.toDouble()),
                pose.heading.toDouble() + turnPower
            )

            var projectedInRestrictedZone = false
            for (field in fields) {
                if (poseInField(projectedPose.toPoint(), field)) {
                    projectedInRestrictedZone = true
                    break
                }
            }

            if (!projectedInRestrictedZone) {
                return null
            }

            val adjustedForwardPower = -forwardPower
            val adjustedStrafePower = -strafePower
            val adjustedTurnPower = -turnPower

            val flVelocity = adjustedForwardPower + adjustedStrafePower + adjustedTurnPower // Front-left wheel
            val frVelocity = adjustedForwardPower - adjustedStrafePower - adjustedTurnPower // Front-right wheel
            val rlVelocity = adjustedForwardPower - adjustedStrafePower // Rear-left wheel
            val rrVelocity = adjustedForwardPower + adjustedStrafePower // Rear-right wheel

            correctionMap["FL"] = flVelocity
            correctionMap["FR"] = frVelocity
            correctionMap["RL"] = rlVelocity
            correctionMap["RR"] = rrVelocity

            return correctionMap
        }




        fun massCreate(fieldList: HashMap<Point, Double>): List<VectorField> {
            val fields = mutableListOf<VectorField>()
            for (field in fieldList) {
                fields.add(VectorField(field.key, field.value))
            }
            return fields
        }

        private fun poseInField(pose: Point, field: VectorField): Boolean {
            val robotRadius = varConfig.robotRadiusAvoidance
            val point = field.point ?: return false
            val y = pose.x ?: return false
            val x = pose.y ?: return false
            val centerX = point.x ?: return false
            val centerY = point.y ?: return false
            val distance = sqrt((x - centerX).pow(2.0) + (y - centerY).pow(2.0))
            return distance < field.rad + robotRadius
        }

        private fun closestFree(pose: Point, field: VectorField): Point {
            val x0 = field.point!!.x!!
            val y0 = field.point!!.y!!
            val x = pose.x!!
            val y = pose.y!!
            val r = field.rad

            // Calculate the unit vector from the field center to the pose
            val dx = x - x0
            val dy = y - y0
            val d = sqrt(dx.pow(2.0) + dy.pow(2.0))
            val ux = dx / d
            val uy = dy / d

            // Calculate the closest point outside the field radius
            val x1 = x0 + r * ux
            val y1 = y0 + r * uy
            return Point(x1, y1)
        }
    }
}