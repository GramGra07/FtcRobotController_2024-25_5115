package org.firstinspires.ftc.teamcode.opModes.auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.gentrifiedApps.velocityvision.enums.CSpace;
import org.gentrifiedApps.velocityvision.pipelines.moa.AssumedBuilder;
import org.gentrifiedApps.velocityvision.pipelines.moa.DetectionBuilder;
import org.gentrifiedApps.velocityvision.pipelines.moa.MeanColorOfAreaDetector;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

@Autonomous(group = GroupingTitles.auto)
@Disabled
public class vvTestAuto extends LinearOpMode {
    private MeanColorOfAreaDetector processor;
    private VisionPortal portal;
    private int detectionNum = 0;

    @Override
    public void runOpMode() {
        processor = new MeanColorOfAreaDetector(
                CSpace.YCrCb,
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
        FtcDashboard.getInstance().startCameraStream(portal, 0.0);
        waitForStart();
    }
}