package org.firstinspires.ftc.teamcode.subsystems

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.extensions.PoseExtensions.toPoint
import org.firstinspires.ftc.teamcode.utilClass.objects.Point
import org.firstinspires.ftc.teamcode.utilClass.objects.VectorField
import org.firstinspires.ftc.teamcode.utilClass.objects.VectorField.Companion.getCorrectionByAvoidancePUSH
import org.firstinspires.ftc.teamcode.utilClass.objects.VectorField.Companion.getCorrectionByAvoidanceSTOP
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.VarConfig

//@Config
class AvoidanceSubsystem {
    enum class AvoidanceTypes {
        PUSH,
        STOP,
        OFF
    }

    companion object {
        var fields: List<VectorField> = VectorField.massCreate(createFields())
        private fun createFields(): HashMap<Point, Double> {
            val fields = hashMapOf<Point, Double>()
            val points = listOf(
                Point(24.0, 0.0),
                Point(48.0, 0.0),
                Point(-24.0, 0.0),
                Point(-48.0, 0.0),
                Point(24.0, -24.0),
                Point(48.0, -24.0),
                Point(-24.0, -24.0),
                Point(-48.0, -24.0)
            )
            for (point in points) {
                fields[point] = rad
            }
            return fields
        }

        var rad: Double = VarConfig.fieldRadius
    }

    init {
        fields = VectorField.massCreate(createFields())
    }


    private var powers: Map<String, Double?>? = null
    private var currentAvoidanceTypes: AvoidanceTypes = AvoidanceTypes.OFF

    fun update(
        localizerSubsystem: LocalizerSubsystem,
        driveSubsystem: DriveSubsystem,
        type: AvoidanceTypes,
    ) {
        updateVars(type)
        when (type) {
            AvoidanceTypes.PUSH -> {
                powers = getCorrectionByAvoidancePUSH(
                    fields,
                    localizerSubsystem.pose().toPoint(),
                )
            }

            AvoidanceTypes.STOP -> {
                powers = getCorrectionByAvoidanceSTOP(
                    fields,
                    localizerSubsystem.pose(),
                    driveSubsystem.leftStickY,
                    driveSubsystem.leftStickX,
                    driveSubsystem.rightStickX
                )
            }

            AvoidanceTypes.OFF -> {}
        }
    }

    fun telemetry(telemetry: Telemetry) {
        telemetry.addData("AVOIDANCE", "")
        telemetry.addData("powers", powers)
        telemetry.addData("Avoidance Type", currentAvoidanceTypes.name)
    }


    private fun updateVars(type: AvoidanceTypes) {
        rad = VarConfig.fieldRadius
        currentAvoidanceTypes = type
    }
}