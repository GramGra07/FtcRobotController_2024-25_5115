package org.firstinspires.ftc.teamcode.subsystems.avoidance

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Vector2d
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.UtilClass.objects.Point
import org.firstinspires.ftc.teamcode.UtilClass.varConfigurations.varConfig
import org.firstinspires.ftc.teamcode.UtilClass.objects.VectorField
import org.firstinspires.ftc.teamcode.UtilClass.objects.VectorField.Companion.calculateRepulsiveForce
import org.firstinspires.ftc.teamcode.UtilClass.objects.VectorField.Companion.getCorrectionByAvoidancePUSH
import org.firstinspires.ftc.teamcode.extensions.PoseExtensions.toPoint
import org.firstinspires.ftc.teamcode.rr.MecanumDrive

@Config
class AvoidanceSubsystem(avoidanceTypes: AvoidanceTypes) {
    enum class AvoidanceTypes {
        PUSH,
        STOP,
        OFF
    }

    @JvmField
    var avoidanceType: AvoidanceTypes
    private var fields: List<VectorField> = VectorField.massCreate(createFields())

    init {
        fields = VectorField.massCreate(createFields())
        this.avoidanceType = avoidanceTypes
    }

    private var rad: Double = varConfig.fieldRadius

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

    var powers: Map<String, Double?>? = null

    fun update(drive: MecanumDrive) {
        updateVars()
        when (avoidanceType) {
            AvoidanceTypes.PUSH -> {
                powers = getCorrectionByAvoidancePUSH(
                    fields,
                    drive.pose.position.toPoint(),
                )
            }

            AvoidanceTypes.STOP -> {
                val repulsiveForce = calculateRepulsiveForce(fields, drive.pose.position.toPoint())
                // Apply the repulsive force to stop the robot
                powers = mapOf(
                    "frontLeft" to -repulsiveForce,
                    "frontRight" to -repulsiveForce,
                    "backLeft" to -repulsiveForce,
                    "backRight" to -repulsiveForce
                )
            }

            AvoidanceTypes.OFF -> {}
        }
    }

    fun telemetry(telemetry: Telemetry) {
        telemetry.addData("addPowers", powers)
        telemetry.addData("Avoidance Type", avoidanceType.name)
    }

    fun draw(packet: TelemetryPacket, drive: MecanumDrive) {
        val rad = rad
        val roboRad = varConfig.robotRadiusAvoidance
        packet.fieldOverlay()
            .setFill("red")
            .setStroke("red")
            .setAlpha(0.3)
        for (field in fields) {
            packet.fieldOverlay()
                .fillCircle(field.point?.y!!, field.point!!.x!!, rad)
        }
        val t = drive.pose
        val halfv: Vector2d = t.heading.vec().times(0.5 * roboRad)
        val p1: Vector2d = t.position.plus(halfv)
        val (x, y) = p1.plus(halfv)
        packet.fieldOverlay()
            .setStrokeWidth(1)
            .setStroke("black")
            .setFill("black")
            .setAlpha(1.0)
            .strokeCircle(t.position.x, t.position.y, roboRad).strokeLine(p1.x, p1.y, x, y)
    }

    private fun updateVars() {
        rad = varConfig.fieldRadius
    }
}