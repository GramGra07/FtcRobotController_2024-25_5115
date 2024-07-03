package org.firstinspires.ftc.teamcode.storage

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.utilClass.drivetrain.Drivetrain

//@Config
class CurrentDrivetrain {
    companion object {
        @JvmField
        var defaultDrivetrain: Int = 0
        var currentDrivetrain: Drivetrain = Drivetrain.drivetrains[defaultDrivetrain]
    }
}