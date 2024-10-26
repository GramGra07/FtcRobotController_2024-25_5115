package org.firstinspires.ftc.teamcode.utilClass.storage

import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance

data class GameStorage(val alliance: Alliance) {
    companion object {
        var alliance: Alliance = Alliance.BLUE
    }
}