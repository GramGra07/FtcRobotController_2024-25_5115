package org.firstinspires.ftc.teamcode.customHardware.autoUtil

import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.StartSide

data class StartLocation(
    var alliance: Alliance,
    var startSide: StartSide,
    var zeros: Boolean = false
)