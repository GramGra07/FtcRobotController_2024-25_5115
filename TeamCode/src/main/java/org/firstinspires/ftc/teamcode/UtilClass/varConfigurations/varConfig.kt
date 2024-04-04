package org.firstinspires.ftc.teamcode.UtilClass.varConfigurations

import com.acmerobotics.dashboard.config.Config

// these variables can be changed from FTC dashboard
@Config
object varConfig {
    @JvmField
    var delay = 1

    @JvmField
    var slowMult = 3

    @JvmField
    var minConfidence = 0.5

    @JvmField
    var minRectArea = 300.0

    @JvmField
    var useFileWriter = true

    @JvmField
    var multipleDrivers = true

    @JvmField
    var usePIDF = true

    @JvmField
    var usingAvoidance = true

    @JvmField
    var fieldRadius = 8.0

    @JvmField
    var correctionSpeedAvoid = 0.02

    @JvmField
    var robotRadiusAvoidance: Double = 8.0
}
