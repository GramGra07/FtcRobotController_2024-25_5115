package org.firstinspires.ftc.teamcode.utilClass.varConfigurations

import com.acmerobotics.dashboard.config.Config

// these variables can be changed from FTC dashboard
@Config
object varConfig {

    @JvmField
    var OTOSxOffset = 1.5

    @JvmField
    var OTOSyOffset = -7.5

    @JvmField
    var OTOShOffset = 0.0

    @JvmField
    var slowMult = 3

    @JvmField
    var useFileWriter = true

    @JvmField
    var multipleDrivers = true


    @JvmField
    var fieldRadius = 8.0

    @JvmField
    var correctionSpeedAvoid = 0.035

    @JvmField
    var robotRadiusAvoidance: Double = 10.0


    @JvmField
    var minRectArea = 300.0
}
