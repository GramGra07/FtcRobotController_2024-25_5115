package org.firstinspires.ftc.teamcode.extensions

import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

object OTOSExtension {
    fun initOTOS(
        hw: HardwareMap,
        name: String,
        startPose: Pose2D? = Pose2D(0.0, 0.0, 0.0)
    ): SparkFunOTOS {
        val t = hw.get(SparkFunOTOS::class.java, name)
        t.setLinearUnit(DistanceUnit.INCH)
        t.setAngularUnit(AngleUnit.DEGREES)
        //For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        val offset = Pose2D(0.0, 0.0, 0.0)
        t.offset = offset
        //For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        t.setLinearScalar(1.0)
        t.setAngularScalar(1.0)
        t.calibrateImu()

        t.resetTracking()

        t.position = startPose
        return t
    }

    fun SparkFunOTOS.getPose(): Pose2D {
        return this.position
    }

    fun Pose2D.toPose(): Pose2d {
        return Pose2d(this.x, this.y, this.h)
    }

    fun SparkFunOTOS.telemetry(telemetry: Telemetry) {
        val pose = this.getPose()
        telemetry.addData("x", pose.x)
        telemetry.addData("y", pose.y)
        telemetry.addData("h", pose.h)
    }
}