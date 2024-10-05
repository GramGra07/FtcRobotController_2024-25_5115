package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles

class ReLocalizationSubsystem(ahwMap: HardwareMap) {
    private var limelight3A: Limelight3A
    var pose: Pose3D? = null
    private val nullPose = Pose3D(
        Position(DistanceUnit.INCH, 0.0, 0.0, 0.0, 0L),
        YawPitchRollAngles(AngleUnit.DEGREES, 0.0, 0.0, 0.0, 0L)
    )

    init {
        limelight3A = ahwMap.get(Limelight3A::class.java, "limelight")
        if (!limelight3A.pipelineSwitch(0)) {
            throw Exception("Not initialized")
        }
        limelight3A.start()
    }

    fun update(
        localizerSubsystem: LocalizerSubsystem,
        relocalize: Boolean = true,
        yaw: Double = localizerSubsystem.heading()
    ) {
        limelight3A.updateRobotOrientation(-yaw)
        limelight3A.getPose()
        if (relocalize) {
            relocalize(localizerSubsystem)
        }
    }

    private fun relocalize(localizerSubsystem: LocalizerSubsystem) {
        if (pose != null) localizerSubsystem.relocalize(pose!!)
    }

    fun Limelight3A.getPose(): Pose3D {
        val m = 39.37
        val res = this.latestResult
        val botpose = res.botpose_MT2
        val dist = res.botposeAvgDist
        val position = botpose.position
        val result = Pose3D(
            Position(
                DistanceUnit.INCH,
                position.x * m,
                position.y * m,
                position.z * m,
                position.acquisitionTime
            ), botpose.orientation
        )
        pose =
            if (result.position.x.toInt() != 0 || ((res.isValid) && res.staleness < 100L) && (dist < 24 * m)) {
                result
            } else {
                null
            }
        return result
    }

    fun telemetry(telemetry: Telemetry) {
        if (pose != null) {
            telemetry.addData("RELOCALIZATION", "")
            telemetry.addData("X", pose!!.position.x)
            telemetry.addData("Y", pose!!.position.y)
            telemetry.addData("H", pose!!.orientation.yaw)
            val status = limelight3A.status
            telemetry.addData(
                "LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.temp, status.cpu, status.fps.toInt()
            )
        }

//        val detectorResults = limelight3A.latestResult.detectorResults
//        for (dr in detectorResults) {
//            telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.className, dr.targetArea)
//        }
    }

}
