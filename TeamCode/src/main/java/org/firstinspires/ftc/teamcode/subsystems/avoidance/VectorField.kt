import org.firstinspires.ftc.teamcode.Point
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.varConfig
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

        fun getCorrectionByAvoidance(fields: List<VectorField>, pose: Point): Map<String, Double>? {
            fieldStrengthCoefficient = varConfig.correctionSpeedAvoid
            var newPose: Point? = null
            for (field in fields) {
                if (poseInField(pose, field)) {
                    newPose = closestFree(pose, field)
                }
            }
            if (newPose == null) {
                return null
            }
            val xError = pose.x!! - newPose.x!!
            val yError = pose.y!! - newPose.y!!
            val theta = atan2(yError, xError)
            val magnitude = sqrt(xError.pow(2.0) + yError.pow(2.0))
            val forwardCorrection = fieldStrengthCoefficient * magnitude * sin(theta) * -1
            val strafeCorrection = fieldStrengthCoefficient * magnitude * cos(theta) * -1
            val flVelocity = forwardCorrection + strafeCorrection  // Front-left wheel
            val frVelocity = forwardCorrection - strafeCorrection * -1 // Front-right wheel
            val rlVelocity = forwardCorrection + strafeCorrection  // Rear-left wheel
            val rrVelocity = forwardCorrection - strafeCorrection * -1 // Rear-right wheel

            return mapOf(
                "FL" to flVelocity,
                "FR" to frVelocity,
                "RL" to rlVelocity,
                "RR" to rrVelocity
            )
        }

        fun massCreate(fieldList: HashMap<Point, Double>): List<VectorField> {
            val fields = mutableListOf<VectorField>()
            for (field in fieldList) {
                fields.add(VectorField(field.key, field.value))
            }
            return fields
        }

        fun poseInField(pose: Point, field: VectorField): Boolean {
            var robotRadius = 8
            val point = field.point ?: return false
            val y = pose.x ?: return false
            val x = pose.y ?: return false
            val centerX = point.x ?: return false
            val centerY = point.y ?: return false
            val distance = sqrt((x - centerX).pow(2.0) + (y - centerY).pow(2.0))
            return distance < field.rad + robotRadius
        }

        private fun closestFree(pose: Point, field: VectorField): Point {
            val point = field.point ?: return Point()
            val x = pose.x ?: return Point()
            val y = pose.y ?: return Point()
            val x0 = point.x ?: return Point()
            val y0 = point.y ?: return Point()
            val r = field.rad
            val d = sqrt((x - x0).pow(2.0) + (y - y0).pow(2.0))
            val x1 = x0 + r * (x - x0) / d
            val y1 = y0 + r * (y - y0) / d
            return Point(x1, y1)
        }
    }
}
