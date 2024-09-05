package org.firstinspires.ftc.teamcode.opModes.tuners.OTOS;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class J5155OTOSHeadingOffsetTuner extends LinearOpMode {

    SparkFunOTOS drive;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = hardwareMap.get(SparkFunOTOS.class, "spark");
        drive.calibrateImu();
        drive.setPosition(new SparkFunOTOS.Pose2D(0.0, 0.0, 0.0));
        drive.setLinearScalar(1.0);
        drive.setAngularScalar(1.0);

        telemetry.addLine("OTOS Heading Offset Tuner");
        telemetry.addLine("Line the side of the robot against a wall and Press START.");
        telemetry.addLine("Then push the robot forward some distance.");
        telemetry.addLine("Finally, copy the heading offset into line 38 of SparkFunOTOSDrive");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Heading Offset (radians, enter this one into SparkFunOTOSDrive!)", Math.atan2(drive.getPosition().y, drive.getPosition().x));
            telemetry.addData("Heading Offset (degrees)", Math.toDegrees(Math.atan2(drive.getPosition().y, drive.getPosition().x)));
            telemetry.update();
        }


    }
}
