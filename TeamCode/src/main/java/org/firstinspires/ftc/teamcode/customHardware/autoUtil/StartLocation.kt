package org.firstinspires.ftc.teamcode.customHardware.autoUtil

import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.StartSide

class StartLocation(alliance: Alliance, startSide: StartSide, zeros:Boolean = false) {
    var alliance: Alliance
    var startSide: StartSide
    var zeros: Boolean

    init {
        this.alliance = alliance
        this.startSide = startSide
        this.zeros = zeros
    }
}