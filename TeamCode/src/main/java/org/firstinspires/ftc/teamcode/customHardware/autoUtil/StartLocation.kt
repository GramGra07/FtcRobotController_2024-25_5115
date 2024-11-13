package org.firstinspires.ftc.teamcode.customHardware.autoUtil

import com.acmerobotics.roadrunner.Pose2d
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance
import org.firstinspires.ftc.teamcode.utilClass.storage.GameStorage
import org.firstinspires.ftc.teamcode.utilClass.storage.PoseStorage

data class StartLocation(
    var alliance: Alliance = GameStorage.alliance,
    var startPose: Pose2d = Pose2d(24.0, 0.0, Math.toRadians(0.0))
) {
    init {
        PoseStorage.currentPose = startPose
        GameStorage.alliance = alliance
    }
    enum class SPOTS{
        RL,RR,BL,BR
    }
    companion object {
        var map: HashMap<SPOTS, Pose2d> =
            hashMapOf(
                SPOTS.RL to Pose2d(12.0, 60.0, Math.toRadians(0.0)),
                SPOTS.RR to Pose2d(-12.0, 60.0, Math.toRadians(0.0)),
                SPOTS.BL to Pose2d(12.0, -60.0, Math.toRadians(180.0)),
                SPOTS.BR to Pose2d(-12.0, -60.0, Math.toRadians(180.0))
            )

        public fun Pose2d.Companion.ofContext(spots: SPOTS): Pose2d {
            return map[spots]!!
        }
    }
}