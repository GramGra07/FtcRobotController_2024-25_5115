package org.firstinspires.ftc.teamcode.subsystems.avoidance

import VectorField
import org.firstinspires.ftc.teamcode.Point
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.varConfig

//@Config
class AvoidanceSubsystem {

    var rad: Double = varConfig.fieldRadius

    private var points: List<Point> = listOf(
        Point(24.0, 0.0),
        Point(48.0, 0.0),
        Point(-24.0, 0.0),
        Point(-48.0, 0.0),
        Point(24.0, -24.0),
        Point(48.0, -24.0),
        Point(-24.0, -24.0),
        Point(-48.0, -24.0)
    )

    private fun createFields(): HashMap<Point, Double> {
        val fields = hashMapOf<Point, Double>()
        for (point in points) {
            fields[point] = rad
        }
        return fields
    }

    var fields: List<VectorField> = VectorField.massCreate(createFields())

    init {
        fields = VectorField.massCreate(createFields())
    }
}