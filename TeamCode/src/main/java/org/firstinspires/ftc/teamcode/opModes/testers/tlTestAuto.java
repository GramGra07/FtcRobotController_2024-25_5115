package org.firstinspires.ftc.teamcode.opModes.testers;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance;
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles;
import org.firstinspires.ftc.teamcode.vision.TargetLock;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(group = GroupingTitles.auto)
@Disabled
public class tlTestAuto extends LinearOpMode {
    private TargetLock processor;
    private VisionPortal portal;

    @Override
    public void runOpMode() {
        processor = new TargetLock(Alliance.BLUE, 20.0);

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(processor)
                .build();
        FtcDashboard.getInstance().startCameraStream(portal, 0.0);
        waitForStart();
    }
}