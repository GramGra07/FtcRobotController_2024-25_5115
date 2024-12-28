package org.firstinspires.ftc.teamcode.customHardware.autoUtil

import com.acmerobotics.roadrunner.Pose2d
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance
import org.firstinspires.ftc.teamcode.utilClass.storage.GameStorage
import org.firstinspires.ftc.teamcode.utilClass.storage.PoseStorage

data class StartLocation(
    var alliance: Alliance = GameStorage.alliance,
    val startPose: Pose2d = Pose2d(0.0, 0.0, 0.0),
) {
    init {
        PoseStorage.currentPose = startPose
        GameStorage.alliance = alliance
    }

    fun build() {
        PoseStorage.currentPose = startPose
        GameStorage.alliance = alliance
    }
}