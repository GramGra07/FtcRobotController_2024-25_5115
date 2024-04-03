package org.firstinspires.ftc.teamcode.subsystems.avoidance

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Point
import org.firstinspires.ftc.teamcode.UtilClass.varConfigurations.varConfig
import org.firstinspires.ftc.teamcode.VectorField
import org.firstinspires.ftc.teamcode.VectorField.Companion.getCorrectionByAvoidancePUSH
import org.firstinspires.ftc.teamcode.extensions.PoseExtensions.toPoint
import org.firstinspires.ftc.teamcode.rr.MecanumDrive

//@Config
class AvoidanceSubsystem(avoidanceTypes: AvoidanceTypes) {
    enum class AvoidanceTypes {
        PUSH,
        STOP
    }

    private var avoidanceType: AvoidanceTypes

    init {
        this.avoidanceType = avoidanceTypes
    }

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

    var powers: Map<String, Double?>? = null

    fun update(drive: MecanumDrive) {
        if (avoidanceType == AvoidanceTypes.PUSH) {
            powers = getCorrectionByAvoidancePUSH(
                fields,
                drive.pose.position.toPoint(),
            )
        } else if (avoidanceType == AvoidanceTypes.STOP) {

        }
    }

    fun telemetry(telemetry: Telemetry) {
        telemetry.addData("addPowers", powers)
        telemetry.addData("Avoidance Type", avoidanceType.name)
    }
}