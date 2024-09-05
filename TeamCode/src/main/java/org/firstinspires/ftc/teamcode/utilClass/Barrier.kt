package org.firstinspires.ftc.teamcode.utilClass

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import org.firstinspires.ftc.teamcode.utilClass.objects.Point

data class Barrier(var location: Point, var radius: Double, var shape: SHAPE, var color: COLOR) {
    enum class SHAPE {
        CIRCLE,
        SQUARE
    }

    enum class COLOR {
        RED,
        BLUE,
        BLACK,
        YELLOW,
    }

    fun draw(packet: TelemetryPacket) {
        val fieldOverlay = packet.fieldOverlay()
        fieldOverlay.setStroke(color.name.lowercase())
        when (shape) {
            SHAPE.CIRCLE -> fieldOverlay.strokeCircle(location.x!!, location.y!!, radius)
            SHAPE.SQUARE -> fieldOverlay.strokeRect(location.x!!, location.y!!, radius, radius)
        }
    }
}