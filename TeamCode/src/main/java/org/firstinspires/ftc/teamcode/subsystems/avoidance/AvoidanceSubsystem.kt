package org.firstinspires.ftc.teamcode.subsystems.avoidance

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Vector2d
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.extensions.PoseExtensions.toPoint
import org.firstinspires.ftc.teamcode.followers.rr.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.utilClass.objects.Point
import org.firstinspires.ftc.teamcode.utilClass.objects.VectorField
import org.firstinspires.ftc.teamcode.utilClass.objects.VectorField.Companion.getCorrectionByAvoidancePUSH
import org.firstinspires.ftc.teamcode.utilClass.objects.VectorField.Companion.getCorrectionByAvoidanceSTOP
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.varConfig

//@Config
class AvoidanceSubsystem {
    enum class AvoidanceTypes {
        PUSH,
        STOP,
        OFF
    }

    private var fields: List<VectorField> = VectorField.massCreate(createFields())

    init {
        fields = VectorField.massCreate(createFields())
    }

    private var rad: Double = varConfig.fieldRadius
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

    var powers: Map<String, Double?>? = null
    private var currentAvoidanceTypes: AvoidanceTypes = AvoidanceTypes.OFF

    fun update(
        driveSubsystem: DriveSubsystem,
        type: AvoidanceTypes,
        drive: MecanumDrive = driveSubsystem.drive
    ) {
        updateVars(type)
        when (type) {
            AvoidanceTypes.PUSH -> {
                powers = getCorrectionByAvoidancePUSH(
                    fields,
                    drive.pose.position.toPoint(),
                )
            }

            AvoidanceTypes.STOP -> {
                powers = getCorrectionByAvoidanceSTOP(
                    fields,
                    drive.pose,
                    driveSubsystem.leftStickY,
                    driveSubsystem.leftStickX,
                    driveSubsystem.rightStickX
                )
            }

            AvoidanceTypes.OFF -> {}
        }
    }

    fun telemetry(telemetry: Telemetry) {
        telemetry.addData("powers", powers)
        telemetry.addData("Avoidance Type", currentAvoidanceTypes.name)
    }

    fun draw(packet: TelemetryPacket, drive: MecanumDrive) {
        val rad = rad
        val roboRad = 8.0
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

    private fun updateVars(type: AvoidanceTypes) {
        rad = varConfig.fieldRadius
        currentAvoidanceTypes = type
    }
}