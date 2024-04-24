package org.firstinspires.ftc.teamcode.utilClass.objects

import com.acmerobotics.roadrunner.Pose2d

class Point(var x: Double? = 0.0, var y: Double? = 0.0) {
    fun setPoint(x: Double, y: Double) {
        this.x = x
        this.y = y
    }

    fun toPose(heading: Double? = 0.0): Pose2d {
        return Pose2d(x!!, y!!, Math.toRadians(heading!!))
    }

    override fun toString(): String {
        return "x: $x, y: $y"
    }
}