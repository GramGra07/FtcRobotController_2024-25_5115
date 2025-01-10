package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(-8.0, -63.0, Math.toRadians(90.0));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setStartPose(startPose)
                .setDimensions(12, 17)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        Pose2d blueStartLeft = new Pose2d(-8.0, 63.0, Math.toRadians(-90.0));
        Pose2d blueStartRight = new Pose2d(8.0, 63.0, Math.toRadians(-90.0));
        Pose2d redStartLeft = new Pose2d(-8.0, -63.0, Math.toRadians(90.0));
        Pose2d redStartRight = new Pose2d(8.0, -63.0, Math.toRadians(90.0)); // used to be 135-72
        Pose2d blueEndLeft = new Pose2d(-24.0, 10.0, Math.toRadians(180.0));
        Pose2d blueEndRight = new Pose2d(-24.0, -10.0, Math.toRadians(180.0));
        Pose2d redEndLeft = new Pose2d(24.0, 10.0, Math.toRadians(0.0));
        Pose2d redEndRight = new Pose2d(24.0, -10.0, Math.toRadians(0.0));
        Pose2d blueHuman = new Pose2d(-46.0, 54.0, Math.toRadians(90.0));
        Pose2d redHuman = new Pose2d(46.0, -54.0, Math.toRadians(-90.0));
        Pose2d blueBasket = new Pose2d(50.0, 50.0, Math.toRadians(135.0));
        Pose2d redBasket = new Pose2d(-50.0, -50.0, Math.toRadians(45.0));
        Pose2d blueSpecimen = new Pose2d(0.0, 36.0, blueStartRight.heading.toDouble());
        Pose2d redSpecimen = new Pose2d(0.0, -36.0, redStartRight.heading.toDouble());
        Pose2d blueSample = new Pose2d(-60.0, 12.0, Math.toRadians(0.0));
        Pose2d redSample = new Pose2d(60.0, -12.0, Math.toRadians(0.0));
        Pose2d blueNeutralSample = new Pose2d(56.0, 12.0, blueStartRight.heading.toDouble());
        Pose2d redNeutralSample = new Pose2d(-58.0, -40, redStartRight.heading.toDouble());

        myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                .lineToY(redHuman.position.y + 10)
                .setTangent(Math.toRadians(-90.0))
                .splineToLinearHeading(
                        new Pose2d(36.0, -24.0, redSample.heading.toDouble()),
                        redSpecimen.heading.toDouble()
                )
                .splineToConstantHeading(
                        new Vector2d(redSample.position.x - 16.0, redSample.position.y),
                        redSample.heading.toDouble()
                )
                .setTangent(redSpecimen.heading.toDouble())
                .lineToY(redHuman.position.y)
                .lineToY(redHuman.position.y + 10)
                .strafeToLinearHeading(
                        new Vector2d(redHuman.position.x, redHuman.position.y + 8),
                        redHuman.heading.toDouble()
                )
                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}