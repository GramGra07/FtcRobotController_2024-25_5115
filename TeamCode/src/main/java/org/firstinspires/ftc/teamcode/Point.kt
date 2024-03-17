package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d

class Point(var x: Double? = 0.0, var y: Double? = 0.0) {
    fun setPoint(x: Double, y: Double) {
        this.x = x
        this.y = y
    }

    fun toPose(): Pose2d {
        return Pose2d(x!!, y!!)
    }
}