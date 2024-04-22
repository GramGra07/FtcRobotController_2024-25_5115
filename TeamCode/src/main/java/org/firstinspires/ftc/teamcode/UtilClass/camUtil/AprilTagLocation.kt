package org.firstinspires.ftc.teamcode.UtilClass.camUtil

import com.acmerobotics.roadrunner.Vector2d
import org.firstinspires.ftc.teamcode.UtilClass.objects.Point

class AprilTagLocation(val location: Point) {

    fun toVector(): Vector2d {
        return Vector2d(this.location.x!!, this.location.y!!)
    }
}