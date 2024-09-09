package org.firstinspires.ftc.teamcode.subsystems

//import org.firstinspires.ftc.teamcode.followers.rr.MecanumDrive
import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.extensions.PoseExtensions.toPose
import org.firstinspires.ftc.teamcode.extensions.PoseExtensions.toPose2d
import org.firstinspires.ftc.teamcode.extensions.PoseExtensions.toString2
import org.firstinspires.ftc.teamcode.followers.pedroPathing.localization.PoseUpdater
import org.firstinspires.ftc.teamcode.followers.pedroPathing.localization.localizers.OTOSLocalizer
import org.firstinspires.ftc.teamcode.storage.DistanceStorage
import org.firstinspires.ftc.teamcode.storage.PoseStorage
import kotlin.math.sqrt


//@Config
class LocalizerSubsystem(ahwMap: HardwareMap, pose: Pose2d) {

    private lateinit var poseUpdater: PoseUpdater

    init {
        poseUpdater = PoseUpdater(ahwMap, OTOSLocalizer(ahwMap, pose.toPose()))
        poseUpdater.pose = pose.toPose()
        reset()
    }

    private var thisDist = 0.0
    private var lastTime = 0.0
    private var currentSpeed: Double = 0.0

    private fun updateDistTraveled(before: Pose2d, after: Pose2d, timer: Double) {
        val deltaX = after.position.x - before.position.x
        val deltaY = after.position.y - before.position.y
        val dist = sqrt(deltaX * deltaX + deltaY * deltaY)
        val deltaTime = timer - lastTime
        lastTime = timer
        currentSpeed = (dist / deltaTime) * 0.0568
        thisDist += dist
        DistanceStorage.totalDist += dist
    }

    private fun reset() {
        thisDist = 0.0
    }

    fun update(
        timer: ElapsedTime?,
    ) {
        poseUpdater.update()

        if (timer != null) {
            updateDistTraveled(PoseStorage.currentPose, this.pose(), timer.seconds())
//            FileWriterFTC.writeToFile(
//                HardwareConfig.fileWriter,
//                this.x().toInt(),
//                this.y().toInt()
//            )
        }
        PoseStorage.currentPose = this.pose()
    }

    fun setPose(pose: Pose2d) {
        poseUpdater.pose = pose.toPose()

    }

    fun telemetry(telemetry: Telemetry) {
        telemetry.addData("LOCALIZATION", "")
        telemetry.addData("Using", "PP SparkFunOTOS")
        telemetry.addData("Pose: ", this.pose().toString2())
        telemetry.addData("totalDistance (in)", "%.1f", DistanceStorage.totalDist)
        telemetry.addData("Current Speed (mph)", "%.1f", currentSpeed)
    }

    fun heading(): Double {
        return poseUpdater.pose.heading
    }

    fun pose(): Pose2d {
        return poseUpdater.pose.toPose2d()

    }

    fun x(): Double {
        return pose().position.x
    }

    fun y(): Double {
        return pose().position.y
    }
}