package org.firstinspires.ftc.teamcode.UtilClass.varStorage

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
    var fieldRadius = 15.0

    @JvmField
    var correctionSpeedAvoid = 0.1
}
