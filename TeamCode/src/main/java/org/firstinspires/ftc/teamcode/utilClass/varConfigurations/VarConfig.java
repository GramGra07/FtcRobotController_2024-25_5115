package org.firstinspires.ftc.teamcode.utilClass.varConfigurations;

import com.acmerobotics.dashboard.config.Config;

// these variables can be changed from FTC dashboard
@Config
public class VarConfig {
    public static boolean loopSaver = false;
    public static boolean autoRotateClaw = false;

    public static double OTOSxOffset = 0.0;//8.32

    public static double OTOSyOffset = 0.0;//-0.7689

    public static double OTOShOffset = 0.0;//0.744 //-1.55

    public static double linearScalar = 1.31; //3.97

    public static double slowMult = 3;

    public static boolean multipleDrivers = false;

    public static boolean relocalize = true;

    public static double fieldRadius = 10.0;
    public static double minRectArea = 300;
    public static double correctionSpeedAvoid = 0.01;
    public static double robotRadiusAvoidance = 8.0;
}
