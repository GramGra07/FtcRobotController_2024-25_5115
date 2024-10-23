//package org.firstinspires.ftc.teamcode.ggutil.psoPID;
//
//import static java.lang.Math.abs;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import java.util.ArrayList;
//
//import ArmSpecific.ArmAngle;
//import CommonUtilities.Models;
//import CommonUtilities.RemoveOutliers;
//
//@TeleOp(name = "FrictionTest", group = "Linear OpMode")
//public class FrictionTest extends LinearOpMode {
//
//    @Override
//    public void runOpMode() {
//        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
//
//        Constants constants = new Constants();
//
//        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, constants.motorName);
//        motor.setDirection(constants.motorDirection);
//        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motor.setPower(0.0);
//
//        ArmAngle armAngle = new ArmAngle(constants.motor, constants.testingAngle.getStart());
//
//        ElapsedTime timer = new ElapsedTime();
//        ArrayList<Double> RPMS = new ArrayList<>();
//        double angularAccel;
//        int lastPosition = 0;
//        double angularVelocity = 0.0;
//        double lastAngle = 0.0;
//        double lastVelocity = 0.0;
//        ArrayList<Double> angularAccelerationData = new ArrayList<>();
//        ArrayList<Double> motorPowers = new ArrayList<>();
//
//        waitForStart();
//        if (!opModeInInit()) {
//            timer.reset();
//        }
//        while (opModeIsActive()) {
//            telemetry.addLine("Please rotate your robot so that gravity does not affect your mechanism");
//
//            // Running motor at half speed
//            double angle = armAngle.findAngle(motor.getCurrentPosition());
//            //todo double angle = get voltage and convert to Radians if using an absolute encoder
//
//            // Run motor
//            motor.setPower(
//                    (angle >= constants.testingAngle.getStart() && angle <= constants.testingAngle.getTarget()) ? 0.5 : 0.0
//            );
//
//            // Measure RPM
//            double ticksPerRevolution = constants.motor.getEncoderTicksPerRotation(); // Encoder resolution (ticks per revolution)
//            double rpm = ((motor.getCurrentPosition() - lastPosition) / ticksPerRevolution) * (60.0 / timer.seconds());
//            lastPosition = motor.getCurrentPosition();
//            double theoreticalRpmMeasured = constants.motor.getRpm() / 2;
//            if (rpm >= theoreticalRpmMeasured * 0.5 && rpm <= theoreticalRpmMeasured * 1.5) {
//                RPMS.add(rpm);
//            }
//            else telemetry.addLine("Rpm Constants is incorrect, or your robot is struggling with the amount of weight it has");
//            double sum = 0;
//            // Make sure size is not returning something other than 0
//            if (!RPMS.isEmpty()) {
//                for (double num : RPMS) sum += num* 2;
//                telemetry.addData("Motor RPM", sum / RPMS.size());
//            }
//
//            // Finding Angular Acceleration
//            if (timer.seconds() != 0.0) {
//                angularVelocity = (angle - lastAngle) / timer.seconds();
//                angularAccel = abs((angularVelocity - lastVelocity) / timer.seconds());
//                if (motor.getPower() != 0.0) {
//                    angularAccelerationData.add(angularAccel);
//                    motorPowers.add(motor.getPower());
//                }
//            }
//
//            // Calculate if friction test is complete and find rotational Inertia
//            if ((angle < constants.testingAngle.getStart() || angle > constants.testingAngle.getTarget()) && motor.getPower() == 0.0) {
//                angularAccelerationData = RemoveOutliers.removeOutliers(angularAccelerationData);
//                motorPowers = RemoveOutliers.removeOutliers(motorPowers);
//
//                ArrayList<Double> rotationalInertias = new ArrayList<>();
//
//                for (int i = 0; i < angularAccelerationData.size(); i++) {
//                    double rotationalInertia = Models.calculateTmotor(
//                            angularAccelerationData.get(i),
//                            constants.motor,
//                            sum / RPMS.size()
//                    ) / motorPowers.get(i);
//                    if (rotationalInertia >= 0.0) {
//                        rotationalInertias.add(rotationalInertia);
//                    }
//                }
//
//                double sum2 = 0;
//                rotationalInertias = RemoveOutliers.removeOutliers(rotationalInertias);
//                for (double num : rotationalInertias) sum2 += num;
//
//                telemetry.addData(
//                        "Rotational Inertia: (Add to config)",
//                        sum2/rotationalInertias.size()
//                );
//            }
//
//            lastAngle = angle;
//            lastVelocity = angularVelocity;
//            timer.reset();
//            telemetry.update();
//        }
//    }
//}
