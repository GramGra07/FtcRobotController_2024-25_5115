package org.firstinspires.ftc.teamcode.utilClass.varConfigurations;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class PIDVals {
    public static PIDFCoefficients extendPIDFCo = new PIDFCoefficients(0.013, 0.0, 0.0000, 0.0);
    public static PIDFCoefficients pitchPIDFCo = new PIDFCoefficients(0.0017, 0.0, 0.00005, 0.00018);
    public static double pitchFCollapse = 0.00015;
    public static double pitchFExtend = 0.0003;
}
