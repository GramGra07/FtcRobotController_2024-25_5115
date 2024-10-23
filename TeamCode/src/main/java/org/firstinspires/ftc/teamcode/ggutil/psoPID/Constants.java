//package org.firstinspires.ftc.teamcode.ggutil.psoPID;
//
//import static java.lang.Math.PI;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//import java.util.ArrayList;
//import java.util.Arrays;
//
//import ArmSpecific.GravityModelConstants;
//import ArmSpecific.Hardware;
//import ArmSpecific.SystemConstants;
//import ArmSpecific.pso4Arms;
//import CommonUtilities.AngleRange;
//import CommonUtilities.PIDFParams;
//import CommonUtilities.PIDFcontroller;
//
//@Config
//public class Constants {
//
//    //CONFIGURATIONS - do before running anything
//
//    //todo Assign Your Motor Specs, direction, and CONFIG Name
//    public final Hardware.Motor motor = new Hardware.Motor(4000,8192,.105,99.225); // for geared motors new Hardware.Motor(RPM,EncTicksPerRev,StallTorque,GearRatio);
//    public final DcMotorSimple.Direction motorDirection = DcMotorSimple.Direction.REVERSE;
//    public final String motorName = "pitchMotor";
//
//
//    //todo provide the angles (in radians) that your arm can run to when testing (larger range the better)
//    static double stationaryAngle = Math.toRadians(3.0);
//    public final AngleRange testingAngle = new AngleRange(stationaryAngle, PI/2);
//    //todo provide angles (in radians) that present as obstacles to the system. If none set to null
//    public final AngleRange obstacle = new AngleRange(-0.1 * PI, -0.3 * PI); // = null;
//
//
//    //TESTING
//
//    //todo change from FRICTION OPMODE results
//    public static double frictionRPM = 74.9;
//    public static double inertiaValue = 1.170751047881278;
//
//
//    //todo change from Gravity OPMODE & Desmos Graph
//    public static double gravityA = -8.74869;
//    public static double gravityB = 1.59221;
//    public static double gravityK = 21.6828;
//
//
//    public static final ArrayList<AngleRange> angleRanges = new ArrayList<AngleRange>() {{
//        add(new AngleRange(stationaryAngle, PI*.5));
//        add(new AngleRange(PI*.5, -PI*.9));
//        add(new AngleRange(-PI*.9, PI*.5));
//        add(new AngleRange(PI*.5, stationaryAngle));
//    }};
//
//    //todo LAST STEP - RUN the test in the TEST MODULE -> TeamCode/src/test/java/org.firstinspires.ftc.teamcode/FindConstants.java
//
//    public static ArrayList<PIDFParams> params = new ArrayList<>(Arrays.asList(
//            new PIDFParams(1.8446998254838274, 0.645459144659821, 0.27580131796119006, 0.08410433897243955),
//            new PIDFParams(2.1017055627786365, 1.5581383742794008, 0.39075904279895557, 2.1556704344429276),
//            new PIDFParams(2.272228025466264, 1.2239629034613366, 0.3078717768561709, 0.6871836151631764),
//            new PIDFParams(3.7913956820669483, 0.02323118751542631, 0.3877392787120557, 0.31197681044323883)
//    ));
//
//
//
//    SystemConstants constant = new SystemConstants(
//            frictionRPM,
//            motor,
//            new GravityModelConstants(gravityA, gravityB, gravityK),
//            inertiaValue
//    );
//    public pso4Arms sim = new pso4Arms(constant, angleRanges, 1.2, obstacle, 3.5);
//    public static boolean gravityRecord = false;
//    public static boolean gravityDisplayDataPoints = false;
//    public static double gravityMotorPower = 0.0;
//    public PIDFcontroller pidfController = new PIDFcontroller(
//            new PIDFParams(0.0, 0.0, 0.0, 0.0),
//            motor,
//            obstacle,
//            Math.toRadians(3.0)
//    );
//
//}
