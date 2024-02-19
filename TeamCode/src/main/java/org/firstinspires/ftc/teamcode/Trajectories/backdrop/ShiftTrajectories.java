package org.firstinspires.ftc.teamcode.Trajectories.backdrop;

public class ShiftTrajectories {
    public static int leftShift = 8;
    public static int rightShift = -8;

//    public static TrajectorySequence shiftLeft(MecanumDrive drive) {
//        updatePose(drive);
//        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
//                .addDisplacementMarker(() -> {
//                    ServoUtil.closeClaw(claw2);
//                    if (StartPose.alliance == Alliance.BLUE && StartPose.side == StartSide.LEFT) {
//                        strafe = 1;
//                    }
//                })
//                .forward(fwd - leftOffset)
//                .strafeLeft(strafe)
//                .build();
//    }
//
//    public static TrajectorySequence shiftRight(MecanumDrive drive) {
//        updatePose(drive);
//        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
//                .addDisplacementMarker(() -> {
//                    ServoUtil.closeClaw(claw2);
//                })
//                .forward(fwd)
//                .strafeRight(12)
//                .build();
//    }
}
