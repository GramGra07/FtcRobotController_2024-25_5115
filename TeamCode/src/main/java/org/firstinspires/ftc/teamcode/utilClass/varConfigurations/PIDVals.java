package org.firstinspires.ftc.teamcode.utilClass.varConfigurations;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class PIDVals {
    public static PIDFCoefficients liftPIDFCo = new PIDFCoefficients(0.01, 0.0, 0.0, 0.0);
    public static PIDFCoefficients pitchPIDFCo = new PIDFCoefficients(0.01, 0.0, 0.0, 0.0);
    public static PIDFCoefficients extendPIDFCo = new PIDFCoefficients(0.01, 0.0, 0.0, 0.0);
}
