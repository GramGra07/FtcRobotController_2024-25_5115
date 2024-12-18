package org.firstinspires.ftc.teamcode.utilClass.varConfigurations;

import com.acmerobotics.dashboard.config.Config;

// these variables can be changed from FTC dashboard
@Config
public class VarConfig {
    public static double pitch = 400.0;
    public static double extend = 400.0;
    public static boolean loopSaver = false;
    public static boolean autoRotateClaw = true;
    public static double slowMult = 3;

    public static boolean relocalize = true;

    public static double fieldRadius = 10.0;
    public static double minRectArea = 300;
    public static double correctionSpeedAvoid = 0.01;
    public static double robotRadiusAvoidance = 8.0;
}
