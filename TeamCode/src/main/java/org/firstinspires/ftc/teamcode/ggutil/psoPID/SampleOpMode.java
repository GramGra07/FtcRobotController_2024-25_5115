//package org.firstinspires.ftc.teamcode.ggutil.psoPID;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//
//import java.util.ArrayList;
//
//import CommonUtilities.AngleRange;
//
//@TeleOp(name = "SampleOpMode", group = "Linear OpMode")
//public class SampleOpMode extends LinearOpMode {
//
//    @Override
//    public void runOpMode() {
//        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
//
//        Constants constants = new Constants();
//
//        // Code executed when initialized
//        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, constants.motorName);
//        motor.setDirection(constants.motorDirection);
//        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motor.setPower(0.0);
//
//        ArrayList<AngleRange> targets = Constants.angleRanges;
//        AngleRange target = targets.get(0);
//        ElapsedTime timer = new ElapsedTime();
//
//        constants.pidfController.resetConstantsAndTarget(Constants.params.get(0), target);
//
//        int x =0;
//
//        while (opModeInInit()) timer.reset();
//        waitForStart();
//
//        while (opModeIsActive() && !isStopRequested()) {
//            double looptime = timer.seconds();
//            timer.reset();
//
//            if (gamepad1.a){
//                x+=1;
//                if(targets.size()!= x) {
//                    target = targets.get(x);
//                }
//            }
//
//
//            for (int i = 0; i < targets.size(); i++) {
//                if (target == targets.get(i)) {
//                    constants.pidfController.resetConstantsAndTarget(Constants.params.get(i), target);
//                    break;
//                }
//            }
//
//            motor.setPower(constants.pidfController.calculateMotorPower(motor.getCurrentPosition(), looptime));
//            telemetry.update();
//        }
//    }
//}
////TODO HAVE accuracy be relative to robot voltage
///*
//
//boolean initalized = false;
//        boolean stop = false;
//ElapsedTime wait = new ElapsedTime();
//if (!stop) {
//                for (int i = 0; i < targets.size(); i++) {
//                    if (constants.pidfController.targetReached(motor.getCurrentPosition(), 5.0)) {
//                        if (targets.size() > i) {
//                            if (!initalized) {
//                                wait.reset();
//                                initalized = true;
//                            }
//
//                            if (wait.seconds() >= 1.5) {
//                                target = targets.get(i + 1);
//                                i++;
//                                initalized = false;
//                                telemetry.addData("Switched to target", i);
//                            }
//
//                        } else {
//                            stop = true;
//                        }
//                        telemetry.addLine("Reached target");
//                    }
//                    if (target == targets.get(i)) {
//                        constants.pidfController.resetConstantsAndTarget(Constants.params.get(i), target);
//                        break;
//                    }
//                }
//            }
// */
