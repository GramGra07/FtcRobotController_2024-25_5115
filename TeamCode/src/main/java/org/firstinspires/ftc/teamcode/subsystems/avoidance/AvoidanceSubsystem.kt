package org.firstinspires.ftc.teamcode.subsystems.avoidance

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import org.firstinspires.ftc.teamcode.Point

class AvoidanceSubsystem {

    private val rad: Double = 10.0

    @JvmField
    var points: List<Point> = listOf(
        Point(0.0, 0.0),
        Point(0.0, 10.0),
        Point(10.0, 0.0),
        Point(10.0, 10.0)
    )

    private fun createFields(): HashMap<Point, Double> {
        val fields = hashMapOf<Point, Double>()
        for (point in points) {
            fields[point] = rad
        }
        return fields
    }

    var fields: List<VectorField> = VectorField.massCreate(createFields())

    private val packet = TelemetryPacket();
    private val dashboard: FtcDashboard = FtcDashboard.getInstance();

    init {
        fields = VectorField.massCreate(createFields())
        packet.fieldOverlay()
            .setFill("red")
            .setAlpha(0.5)
        for (field in fields) {
            packet.fieldOverlay()
                .fillCircle(field.point.x!!, field.point.y!!, rad)
        }
        dashboard.sendTelemetryPacket(packet);
    }

    fun update() {
        packet.fieldOverlay()
            .setFill("red")
            .setAlpha(0.5)
        for (field in fields) {
            packet.fieldOverlay()
                .fillCircle(field.point.x!!, field.point.y!!, rad)
        }
        dashboard.sendTelemetryPacket(packet)
    }
}