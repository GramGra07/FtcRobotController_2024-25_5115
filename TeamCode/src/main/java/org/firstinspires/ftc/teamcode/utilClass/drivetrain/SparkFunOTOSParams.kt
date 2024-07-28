package org.firstinspires.ftc.teamcode.utilClass.drivetrain

import com.qualcomm.hardware.sparkfun.SparkFunOTOS

data class SparkFunOTOSParams(
    var name: String,
    var offset: SparkFunOTOS.Pose2D,
    var linearScalar: Double,
    var angularScalar: Double,
)