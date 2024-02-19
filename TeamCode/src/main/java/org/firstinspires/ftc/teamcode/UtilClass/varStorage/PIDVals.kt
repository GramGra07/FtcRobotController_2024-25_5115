package org.firstinspires.ftc.teamcode.UtilClass.varStorage

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.PIDFCoefficients

@Config
object PIDVals {
    var extensionPIDFCo = PIDFCoefficients(0.01, 0.0, 0.0, 0.0)
    var rotationPIDFCo = PIDFCoefficients(0.0005, 0.0, 0.0, 0.0001)
}
