package org.firstinspires.ftc.teamcode.utilClass.drawer

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D
import org.firstinspires.ftc.teamcode.subsystems.LocalizerSubsystem
import kotlin.math.cos
import kotlin.math.sin

class Drawing {

//    private fun drawReLocalizer(packet: TelemetryPacket) {
//        val fieldOverlay = packet.fieldOverlay()
//        ATLocations.allLocations.forEach { (id, locationData) ->
//            val location = locationData.location
//            if (localizingID.contains(id)) {
//                fieldOverlay.setStroke("green").setAlpha(1.0)
//            } else if (currentSeenID.contains(id)) {
//                fieldOverlay.setStroke("orange").setAlpha(1.0)
//            } else {
//                fieldOverlay.setStroke("blue").setAlpha(0.5)
//            }
//            fieldOverlay.strokeRect(location.y!!, location.x!!, 0.5, 0.5)
//        }
//
//    }

    fun drawLocalization(packet: TelemetryPacket, localizerSubsystem: LocalizerSubsystem) {
        val fieldOverlay = packet.fieldOverlay()
        val roboRad = 8.0
        val color = "purple"
        val l = localizerSubsystem.pose()
        val h2 = l.heading.toDouble()
        val half2 = roboRad / 2
        val cos2 = cos(h2)
        val sin2 = sin(h2)
        val p1s2 = Pose2D(l.x + (sin2 * half2), l.y + (cos2 * half2), 0.0)
        val newS2 = Pose2D(l.x + (sin2 * roboRad), l.y + (cos2 * roboRad), 0.0)

        fieldOverlay
            .setStroke(color)
            .setFill(color)
            .strokeCircle(l.x, l.y, roboRad)
            .strokeLine(p1s2.x, p1s2.y, newS2.x, newS2.y)
    }

    companion object {
        fun drawAll(
            packet: TelemetryPacket,
            dashboard: FtcDashboard,
            localizerSubsystem: LocalizerSubsystem
        ) {

            Drawing().drawLocalization(packet, localizerSubsystem)
            dashboard.sendTelemetryPacket(packet)
        }
    }
}