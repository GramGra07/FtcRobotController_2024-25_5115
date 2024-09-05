package org.firstinspires.ftc.teamcode.opModes.tuners.OTOS;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class J5155OTOSAngularScalar extends LinearOpMode {

    SparkFunOTOS drive;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = hardwareMap.get(SparkFunOTOS.class, "spark");
        drive.calibrateImu();
        drive.setPosition(new SparkFunOTOS.Pose2D(0.0, 0.0, 0.0));
        drive.setLinearScalar(1.0);
        drive.setAngularScalar(1.0);
        double radsTurned = 0;
        Double lastHeading = 0.0;
//        Rotation2d lastHeading = Rotation2d.fromDouble(0);
        telemetry.addLine("OTOS Angular Scalar Tuner");
        telemetry.addLine("Press START, then rotate the robot on the ground 10 times (3600 degrees).");
        telemetry.addLine("Then copy the scalar into SparkFunOTOSDrive.");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            radsTurned += drive.getPosition().h - lastHeading;
            lastHeading = drive.getPosition().h;
            telemetry.addData("Uncorrected Degrees Turned", Math.toDegrees(radsTurned));
            telemetry.addData("Calculated Angular Scalar", 3600 / Math.toDegrees(radsTurned));
            telemetry.update();
        }


    }
}
