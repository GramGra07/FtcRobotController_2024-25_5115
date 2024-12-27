package org.firstinspires.ftc.teamcode.customHardware.autoUtil

import com.acmerobotics.roadrunner.Pose2d
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance
import org.firstinspires.ftc.teamcode.followers.pedroPathing.pathGeneration.Point
import org.firstinspires.ftc.teamcode.utilClass.storage.GameStorage
import org.firstinspires.ftc.teamcode.utilClass.storage.PoseStorage

data class StartLocation(
    var alliance: Alliance = GameStorage.alliance,
    var startPose: Point = Point(0.0, 0.0),
    var startHeading: Double = 0.0
) {
    init {
        PoseStorage.currentPose = Pose2d(startPose.x, startPose.y, startHeading)
        GameStorage.alliance = alliance
    }

    fun build() {
        PoseStorage.currentPose = Pose2d(startPose.x, startPose.y, startHeading)
        GameStorage.alliance = alliance
    }
}