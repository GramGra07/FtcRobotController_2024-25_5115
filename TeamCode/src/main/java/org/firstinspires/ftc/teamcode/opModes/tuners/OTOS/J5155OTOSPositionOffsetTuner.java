package org.firstinspires.ftc.teamcode.opModes.tuners.OTOS;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
public class J5155OTOSPositionOffsetTuner extends LinearOpMode {
    SparkFunOTOS drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = hardwareMap.get(SparkFunOTOS.class, "spark");
        drive.calibrateImu();
        drive.setPosition(new SparkFunOTOS.Pose2D(0.0, 0.0,0.0));
        drive.setLinearScalar(1.0);
        drive.setAngularScalar(1.0);
        telemetry.addLine("OTOS Position Offset Tuner");
        telemetry.addLine("Line the robot against the corner of two walls facing forward and Press START.");
        telemetry.addLine("Then rotate the robot exactly 180 degrees and press it back into the corner.");
        telemetry.addLine("Finally, copy the pose offset into line 38 of SparkFunOTOSDrive.");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Heading (deg)",Math.toDegrees(drive.getPosition().h));
            if (Math.abs(Math.toDegrees(drive.getPosition().h)) > 175) {
                telemetry.addData("X Offset", drive.getPosition().x / 2);
                telemetry.addData("Y Offset", drive.getPosition().y / 2);
            } else {
                telemetry.addLine("Rotate the robot 180 degrees and align it to the corner again.");
            }
            telemetry.update();
        }


    }
}
