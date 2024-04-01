package org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoClasses

import org.firstinspires.ftc.teamcode.startEnums.Alliance
import org.firstinspires.ftc.teamcode.startEnums.StartSide

class StartLocation(alliance: Alliance, startSide: StartSide) {
    var alliance: Alliance
    var startSide: StartSide

    init {
        this.alliance = alliance
        this.startSide = startSide
    }
}