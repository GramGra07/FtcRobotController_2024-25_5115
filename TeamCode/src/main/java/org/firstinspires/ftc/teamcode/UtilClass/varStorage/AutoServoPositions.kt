package org.firstinspires.ftc.teamcode.UtilClass.varStorage

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.backClaw

@Config
object AutoServoPositions {
    //AutoServoPositions.flipUp
    @JvmField
    var flipDown: Int = backClaw

    @JvmField
    var flipUp = 60
}
