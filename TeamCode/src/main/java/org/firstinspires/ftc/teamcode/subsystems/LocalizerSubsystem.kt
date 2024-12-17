package org.firstinspires.ftc.teamcode.subsystems

//import org.firstinspires.ftc.teamcode.followers.rr.MecanumDrive
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
import org.firstinspires.ftc.teamcode.extensions.PoseExtensions.toPose
import org.firstinspires.ftc.teamcode.extensions.PoseExtensions.toPose2d
import org.firstinspires.ftc.teamcode.extensions.PoseExtensions.toString2
import org.firstinspires.ftc.teamcode.followers.pedroPathing.follower.Follower
import org.firstinspires.ftc.teamcode.followers.pedroPathing.localization.Pose
import org.firstinspires.ftc.teamcode.followers.pedroPathing.localization.PoseUpdater
import org.firstinspires.ftc.teamcode.followers.pedroPathing.localization.localizers.TwoWheelLocalizer
import org.firstinspires.ftc.teamcode.followers.pedroPathing.util.Drawing
import org.firstinspires.ftc.teamcode.utilClass.storage.DistanceStorage
import org.firstinspires.ftc.teamcode.utilClass.storage.PoseStorage
import kotlin.math.sqrt


//@Config
class LocalizerSubsystem(ahwMap: HardwareMap, val pose: Pose) {
    lateinit var poseUpdater: PoseUpdater
    lateinit var follower: Follower

    init {
        poseUpdater = PoseUpdater(ahwMap, TwoWheelLocalizer(ahwMap, pose))
        poseUpdater.pose = pose
        follower = Follower(ahwMap)
        follower.setStartingPose(pose)

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
            updateDistTraveled(
                PoseStorage.currentPose.toPose2d(),
                this.pose().toPose2d(),
                timer.seconds()
            )
        }
        PoseStorage.currentPose = this.pose()
    }

    fun setPose(pose: Pose2d) {
        poseUpdater.pose = pose.toPose()
    }

    fun telemetry(telemetry: Telemetry) {
        telemetry.addData("LOCALIZATION", "")
        telemetry.addData("Using", "PP Three")
        telemetry.addData("Pose: ", this.pose().toString2())
        telemetry.addData("totalDistance (in)", "%.1f", DistanceStorage.totalDist)
        telemetry.addData("Current Speed (mph)", "%.1f", currentSpeed)
    }

    fun heading(): Double {
        return poseUpdater.pose.heading
    }

    fun pose(): Pose {
        return poseUpdater.pose
    }

    fun x(): Double {
        return poseUpdater.pose.x
    }

    fun y(): Double {
        return poseUpdater.pose.y
    }

    fun draw(dashboard: FtcDashboard, packet: TelemetryPacket = TelemetryPacket()) {
        dashboard.clearTelemetry()
        packet.field()
        packet.fieldOverlay().setStroke("#e54aa1")
        Drawing.drawRobot(
            pose,
            "#e54aa1",
        )
        dashboard.sendTelemetryPacket(packet)
    }

    fun relocalize(pose: Pose3D) {
        setPose(Pose2d(pose.position.x, pose.position.y, this.heading()))//this.heading()
    }
}