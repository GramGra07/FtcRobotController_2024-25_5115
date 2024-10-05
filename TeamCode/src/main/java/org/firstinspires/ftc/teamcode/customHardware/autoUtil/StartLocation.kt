package org.firstinspires.ftc.teamcode.customHardware.autoUtil

import com.acmerobotics.roadrunner.Pose2d
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance
import org.firstinspires.ftc.teamcode.storage.GameStorage
import org.firstinspires.ftc.teamcode.storage.PoseStorage

data class StartLocation(
    var alliance: Alliance = GameStorage.alliance,
    var startPose: Pose2d = Pose2d(24.0, 0.0, Math.toRadians(0.0))
) {
    init {
        PoseStorage.currentPose = startPose
    }
}