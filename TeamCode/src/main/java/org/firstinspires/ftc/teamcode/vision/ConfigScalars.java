package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

@Config
public class ConfigScalars {
    public static Scalar redLow = new Scalar(0.0, 0, 160);
    public static Scalar redHigh = new Scalar(255, 255.0, 230);
    public static Scalar blueHigh = new Scalar(255, 255, 255);
    public static Scalar blueLow = new Scalar(0.0, 0, 100);

    public static Scalar yellowLow = new Scalar(0, 80.0, 100.0);
    public static Scalar yellowHigh = new Scalar(255, 255.0, 255.0);
}
