package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.actions.teleop.CancelableFollowTrajectoryAction
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig
import org.firstinspires.ftc.teamcode.extensions.PoseExtensions.toPoint
import org.firstinspires.ftc.teamcode.extensions.PoseExtensions.toPose
import org.firstinspires.ftc.teamcode.extensions.PoseExtensions.toPose2d
import org.firstinspires.ftc.teamcode.followers.pedroPathing.localization.PoseUpdater
import org.firstinspires.ftc.teamcode.followers.rr.MecanumDrive
import org.firstinspires.ftc.teamcode.storage.DistanceStorage
import org.firstinspires.ftc.teamcode.storage.PoseStorage
import org.firstinspires.ftc.teamcode.utilClass.FileWriterFTC
import kotlin.math.sqrt


//@Config
class LocalizerSubsystem(ahwMap: HardwareMap, pose: Pose2d, var type: LocalizationType) {
    enum class LocalizationType {
        RR,
        PP
    }

    private lateinit var drive: MecanumDrive
    private lateinit var poseUpdater: PoseUpdater
    lateinit var cancelableFollowing: CancelableFollowTrajectoryAction

    init {
        when (type) {
            LocalizationType.RR -> drive = MecanumDrive(ahwMap, pose)
            LocalizationType.PP -> {
                poseUpdater = PoseUpdater(ahwMap)
                poseUpdater.pose = pose.toPose()
            }
        }
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
        timer: ElapsedTime,
        time: Double = timer.seconds()
    ) {
        when (type) {
            LocalizationType.RR -> drive.updatePoseEstimate()
            LocalizationType.PP -> poseUpdater.update()
        }
        updateDistTraveled(PoseStorage.currentPose, this.pose(), time)
        FileWriterFTC.writeToFile(
            HardwareConfig.fileWriter,
            this.x().toInt(),
            this.y().toInt()
        )

        PoseStorage.currentPose = this.pose()
    }

    fun setPose(pose: Pose2d) {
        when (type){
            LocalizationType.RR -> drive.pose = pose
            LocalizationType.PP -> poseUpdater.pose = pose.toPose()
        }
    }

    fun telemetry(telemetry: Telemetry) {
        telemetry.addData("Pose: ", this.pose().toPoint().toString())
        when (type) {
            LocalizationType.RR -> telemetry.addData("Using", "RoadRunner")
            LocalizationType.PP -> telemetry.addData("Using", "PedroPathing")
        }
        telemetry.addData("totalDistance (in)", "%.1f", DistanceStorage.totalDist)
        telemetry.addData("Current Speed (mph)", "%.1f", currentSpeed)
    }

    fun draw(packet: TelemetryPacket) {
        val color = when (type) {
            LocalizationType.RR -> "blue"
            LocalizationType.PP -> "green"
        }
        val roboRad = 8.0
        val t = this.pose()
        val halfv: Vector2d = t.heading.vec().times(0.5 * roboRad)
        val p1: Vector2d = t.position.plus(halfv)
        val (x, y) = p1.plus(halfv)
        packet.fieldOverlay()
            .setStrokeWidth(1)
            .setStroke(color)
            .setFill(color)
            .setAlpha(1.0)
            .strokeCircle(t.position.x, t.position.y, roboRad).strokeLine(p1.x, p1.y, x, y)
    }

    fun heading(): Double {
        return when (type) {
            LocalizationType.RR -> drive.pose.heading.toDouble()
            LocalizationType.PP -> poseUpdater.pose.heading
        }
    }

    fun pose(): Pose2d {
        return when (type) {
            LocalizationType.RR -> drive.pose
            LocalizationType.PP -> poseUpdater.pose.toPose2d()
        }
    }

    fun x(): Double {
        return pose().position.x
    }

    fun y(): Double {
        return pose().position.y
    }

    fun headingRad(): Double {
        return Math.toRadians(heading())
    }

    fun headingDeg(): Double {
        return heading()
    }

}