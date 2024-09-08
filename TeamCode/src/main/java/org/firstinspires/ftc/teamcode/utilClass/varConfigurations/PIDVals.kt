package org.firstinspires.ftc.teamcode.utilClass.varConfigurations

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.PIDFCoefficients

@Config
object PIDVals {
    @JvmField
    var liftPIDFCo = PIDFCoefficients(0.01, 0.0, 0.0, 0.0)

    @JvmField
    var extensionPIDFCo = PIDFCoefficients(0.01, 0.0, 0.0, 0.0)

    @JvmField
    var extendPIDFCo = PIDFCoefficients(0.01, 0.0, 0.0, 0.0)
}
