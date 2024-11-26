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
import org.firstinspires.ftc.teamcode.followers.pedroPathing.localization.localizers.OTOSLocalizer
import org.firstinspires.ftc.teamcode.followers.roadRunner.Drawing
import org.firstinspires.ftc.teamcode.followers.roadRunner.SparkFunOTOSDrive
import org.firstinspires.ftc.teamcode.utilClass.storage.DistanceStorage
import org.firstinspires.ftc.teamcode.utilClass.storage.PoseStorage
import kotlin.math.sqrt


//@Config
class LocalizerSubsystem(ahwMap: HardwareMap, pose: Pose, val type: LocalizerType) {
    enum class LocalizerType {
        PEDRO,
        ROADRUNNER
    }

    lateinit var poseUpdater: PoseUpdater
    lateinit var follower: Follower
    private lateinit var sparkFunDrive: SparkFunOTOSDrive
    private val startPose = pose

    init {
        when (type) {
            LocalizerType.PEDRO -> {
                poseUpdater = PoseUpdater(ahwMap, OTOSLocalizer(ahwMap, pose))
                poseUpdater.pose = pose
                follower = Follower(ahwMap)
                follower.setStartingPose(pose)
            }

            LocalizerType.ROADRUNNER -> {
                sparkFunDrive = SparkFunOTOSDrive(ahwMap, pose.toPose2d())
                sparkFunDrive.pose = pose.toPose2d()
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
        timer: ElapsedTime?,
    ) {
        when (type) {
            LocalizerType.PEDRO -> {
                poseUpdater.update()
            }

            LocalizerType.ROADRUNNER -> {
                sparkFunDrive.updatePoseEstimate()
            }
        }

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
        when (type) {
            LocalizerType.PEDRO -> {
                poseUpdater.pose = pose.toPose()
            }

            LocalizerType.ROADRUNNER -> {
                PoseStorage.currentPose = pose.toPose()
                sparkFunDrive.pose = pose
            }
        }
    }

    fun telemetry(telemetry: Telemetry) {
        telemetry.addData("LOCALIZATION", "")
        when (type) {
            LocalizerType.PEDRO -> {
                telemetry.addData("Using", "PP OTOS")
            }

            LocalizerType.ROADRUNNER -> {
                telemetry.addData("Using", "RR OTOS")
            }
        }
        telemetry.addData("Pose: ", this.pose().toString2())
        telemetry.addData("totalDistance (in)", "%.1f", DistanceStorage.totalDist)
        telemetry.addData("Current Speed (mph)", "%.1f", currentSpeed)
    }

    fun heading(): Double {
        return when (type) {
            LocalizerType.PEDRO -> {
                poseUpdater.pose.heading
            }

            LocalizerType.ROADRUNNER -> {
                sparkFunDrive.pose.heading.toDouble()
            }
        }
    }

    fun pose(): Pose {
        return when (type) {
            LocalizerType.PEDRO -> {
                poseUpdater.pose
            }

            LocalizerType.ROADRUNNER -> {
                sparkFunDrive.pose.toPose()
            }
        }
    }

    fun x(): Double {
        return when (type) {
            LocalizerType.PEDRO -> {
                poseUpdater.pose.x
            }

            LocalizerType.ROADRUNNER -> {
                sparkFunDrive.pose.position.x
            }
        }
    }

    fun y(): Double {
        return when (type) {
            LocalizerType.PEDRO -> {
                poseUpdater.pose.y
            }

            LocalizerType.ROADRUNNER -> {
                sparkFunDrive.pose.position.y
            }
        }
    }

    fun draw(dashboard: FtcDashboard, packet: TelemetryPacket = TelemetryPacket()) {
        dashboard.clearTelemetry()
        packet.field()
        packet.fieldOverlay().setStroke("#3F51B5")
        Drawing.drawRobot(packet.fieldOverlay(), pose().toPose2d())
        dashboard.sendTelemetryPacket(packet)
    }

    fun relocalize(pose: Pose3D) {
        setPose(Pose2d(pose.position.x, pose.position.y, this.heading()))//this.heading()
    }
}