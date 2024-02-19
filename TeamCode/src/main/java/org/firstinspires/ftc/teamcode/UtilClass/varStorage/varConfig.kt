package org.firstinspires.ftc.teamcode.UtilClass.varStorage

import com.acmerobotics.dashboard.config.Config

// these variables can be changed from FTC dashboard
@Config
object varConfig {
    var delay = 1
    var slowMult = 3
    var minConfidence = 0.5
    var minRectArea = 300.0
    var useFileWriter = true
    var multipleDrivers = false
    var usePIDF = true
}
