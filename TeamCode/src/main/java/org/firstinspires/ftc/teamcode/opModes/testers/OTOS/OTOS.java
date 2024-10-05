package org.firstinspires.ftc.teamcode.opModes.testers.OTOS;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "OTOS TESTER")
//@Disabled
public class OTOS extends LinearOpMode {
    SparkFunOTOS sparkFunOTOS;

    @Override
    public void runOpMode() {
        sparkFunOTOS = hardwareMap.get(SparkFunOTOS.class, "spark");
        sparkFunOTOS.setLinearUnit(DistanceUnit.INCH);
        sparkFunOTOS.setAngularUnit(AngleUnit.DEGREES);
        //For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        sparkFunOTOS.setOffset(new SparkFunOTOS.Pose2D(0.0, 0.0, 0.0));
        //For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        sparkFunOTOS.setLinearScalar(1.0);
        sparkFunOTOS.setAngularScalar(1.0);
        sparkFunOTOS.calibrateImu();

        sparkFunOTOS.resetTracking();

        sparkFunOTOS.setPosition(new SparkFunOTOS.Pose2D(0.0, 0.0, 0.0));

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("X: ", sparkFunOTOS.getPosition().x);
            telemetry.addData("Y: ", sparkFunOTOS.getPosition().y);
            telemetry.addData("H: ", sparkFunOTOS.getPosition().h);
            telemetry.update();
        }
    }
}
