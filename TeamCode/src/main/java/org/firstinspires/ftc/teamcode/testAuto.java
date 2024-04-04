package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.gentrifiedApps.velocityvision.AssumedBuilder;
import org.gentrifiedApps.velocityvision.DetectionBuilder;
import org.gentrifiedApps.velocityvision.MeanColorOfAreaDetector;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

@Autonomous
public class testAuto extends LinearOpMode {
    private MeanColorOfAreaDetector processor;
    private VisionPortal portal;
    private int detectionNum = 0;

    @Override
    public void runOpMode() {
        processor = new MeanColorOfAreaDetector(
                new DetectionBuilder(
                        new Rect(new Point(120.0, 50.0), new Point(230.0, 150.0)),
                        "left",
                        new Scalar(0.0, 140.0, 0.0),
                        new Scalar(255.0, 255.0, 255.0),
                        () -> detectionNum = 1
                ),
                new DetectionBuilder(
                        new Rect(new Point(570.0, 70.0), new Point(680.0, 170.0)),
                        "right",
                        new Scalar(0.0, 140.0, 0.0),
                        new Scalar(255.0, 255.0, 255.0), () -> detectionNum = 2
                ),
                new AssumedBuilder("middle", () -> detectionNum = 3)
        );

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(processor)
                .build();
        waitForStart();
    }
}