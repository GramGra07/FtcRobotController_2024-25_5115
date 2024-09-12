package org.firstinspires.ftc.teamcode.customHardware.autoUtil

import com.acmerobotics.roadrunner.Pose2d
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance
import org.firstinspires.ftc.teamcode.storage.GameStorage
import org.firstinspires.ftc.teamcode.storage.PoseStorage

data class StartLocation(
    var alliance: Alliance = GameStorage.alliance,
    var startPose: Pose2d = PoseStorage.currentPose
){
    init{
        PoseStorage.currentPose = startPose
    }
}