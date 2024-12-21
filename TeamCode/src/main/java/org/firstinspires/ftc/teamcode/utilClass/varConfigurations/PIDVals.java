package org.firstinspires.ftc.teamcode.utilClass.varConfigurations;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class PIDVals {
    public static PIDFCoefficients extendPIDFCo = new PIDFCoefficients(0.013, 0.0, 0.00003, 0.0);
    public static PIDFCoefficients pitchPIDFCo = new PIDFCoefficients(0.0015, 0.0, 0.0, 0.00015);

    public static double pitchFCollapse = 0.00015;
    public static double pitchFExtend = 0.00015;
}
