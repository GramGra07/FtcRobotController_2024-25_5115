package org.firstinspires.ftc.teamcode.utilClass.storage

import com.acmerobotics.roadrunner.Pose2d

class PoseStorage {
    companion object {
        var currentPose: Pose2d = Pose2d(0.0, 0.0, -Math.PI / 2)
    }
}