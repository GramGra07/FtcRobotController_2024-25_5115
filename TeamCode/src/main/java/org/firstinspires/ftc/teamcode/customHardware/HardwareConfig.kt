package org.firstinspires.ftc.teamcode.customHardware

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.customHardware.loopTime.LoopTimeController
import org.firstinspires.ftc.teamcode.customHardware.loopTime.LoopTimeController.Companion.every
import org.firstinspires.ftc.teamcode.followers.pedroPathing.follower.Follower
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.DriverAid
import org.firstinspires.ftc.teamcode.subsystems.LocalizerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.ArmSubsystem
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.ScoringSubsystem
import org.firstinspires.ftc.teamcode.subsystems.humanInput.Drivers
import org.firstinspires.ftc.teamcode.subsystems.humanInput.Drivers.bindDriverButtons
import org.firstinspires.ftc.teamcode.subsystems.humanInput.Drivers.currentFieldCentric
import org.firstinspires.ftc.teamcode.subsystems.humanInput.Drivers.switchProfile
import org.firstinspires.ftc.teamcode.subsystems.humanInput.Operators.bindOtherButtons
import org.firstinspires.ftc.teamcode.utilClass.storage.CurrentDrivetrain
import org.firstinspires.ftc.teamcode.utilClass.storage.PoseStorage
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.VarConfig

open class HardwareConfig(
    private val myOpMode: LinearOpMode,
    auto: Boolean,
    startLocation: StartLocation,
    ahwMap: HardwareMap = myOpMode.hardwareMap
) {

    init {
        initRobot(ahwMap, auto, startLocation)
    }

    lateinit var driveSubsystem: DriveSubsystem
    lateinit var localizerSubsystem: LocalizerSubsystem
    lateinit var driveFollower: Follower

    lateinit var armSubsystem: ArmSubsystem
    lateinit var scoringSubsystem: ScoringSubsystem
    lateinit var driverAid: DriverAid
//    lateinit var reLocalizationSubsystem: ReLocalizationSubsystem

//    lateinit var brush: BrushlandRoboticsSensor
//    lateinit var beamBreakSensor: BeamBreakSensor

    companion object {
        val dt = CurrentDrivetrain.currentDrivetrain

        lateinit var telemetry: Telemetry
        lateinit var dashboard: FtcDashboard
        val timer: ElapsedTime = ElapsedTime()

        const val CAM1 = "Webcam 1"

        lateinit var loopTimeController: LoopTimeController

        var once = false

        private var hasMovedOnInit = false

        var allHubs: List<LynxModule> = ArrayList()
    }

    fun initRobot(
        ahwMap: HardwareMap,
        auto: Boolean,
        startLocation: StartLocation
    ) {
        startLocation.build()
        localizerSubsystem =
            LocalizerSubsystem(
                ahwMap,
                PoseStorage.currentPose,
                LocalizerSubsystem.LocalizerType.PEDRO
            )

        driveSubsystem = DriveSubsystem(ahwMap, localizerSubsystem, dt)

        allHubs = ahwMap.getAll(LynxModule::class.java)
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }

        armSubsystem =
            ArmSubsystem(ahwMap)
        scoringSubsystem =
            ScoringSubsystem(ahwMap, auto, armSubsystem)

        driverAid = DriverAid(scoringSubsystem, armSubsystem, localizerSubsystem)
//        reLocalizationSubsystem =
//            ReLocalizationSubsystem(ahwMap)

        telemetry = myOpMode.telemetry

        dashboard = FtcDashboard.getInstance()
        once = false
        dt.telemetry(telemetry)
        localizerSubsystem.draw(dashboard)
        if (!auto) {
            driveFollower = localizerSubsystem.follower
            loopTimeController = LoopTimeController(timer)
            telemetry.update()
        }
    }

    fun doBulk() {
        bindDriverButtons(
            myOpMode, driveSubsystem,
//            liftSubsystem,
        )
        bindOtherButtons(
            myOpMode,
            scoringSubsystem,
            armSubsystem,
            driverAid
        )
        if (VarConfig.multipleDrivers) {
            switchProfile(myOpMode)
        }
        driveSubsystem.driveByGamepads(
            currentFieldCentric,
            myOpMode,
        )
        driveSubsystem.update()

        localizerSubsystem.update(timer)

        scoringSubsystem.update()

        armSubsystem.update()

//        loopTimeController.every(if (VarConfig.loopSaver) 30 else 10) {
//            reLocalizationSubsystem.update(localizerSubsystem, VarConfig.relocalize)
//        }

        loopTimeController.every(if (VarConfig.loopSaver) 30 else 10) {
            buildTelemetry() //makes telemetry
        }
//
////        if (!VarConfig.loopSaver) {
////            startCameraStream()
////        } else {
////            stopCameraStream()
////        }
//
        localizerSubsystem.draw(dashboard)

        loopTimeController.update()
        moveOnInit(myOpMode, armSubsystem, scoringSubsystem)
    }

    fun once() {
        if (!once) {
            telemetry.clearAll()
            timer.reset()
//            startCameraStream()
            myOpMode.gamepad1.setLedColor(229.0, 74.0, 161.0, -1)
            myOpMode.gamepad2.setLedColor(0.0, 0.0, 0.0, -1)
            once = true
        }
    }

    private fun lynxModules() {
        for (hub in allHubs) {
            hub.clearBulkCache()
        }
    }

    private fun buildTelemetry() {
        loopTimeController.telemetry(telemetry)
        teleSpace()
        if (VarConfig.multipleDrivers) {
            telemetry.addData(
                "Drivers",
                Drivers.currDriver.name.toString() + " " + Drivers.currOther.name.toString()
            )
            teleSpace()
        }
        driveSubsystem.telemetry(telemetry, false)
        teleSpace()
//        reLocalizationSubsystem.telemetry(telemetry)
//        teleSpace()
        localizerSubsystem.telemetry(telemetry)
        teleSpace()
        armSubsystem.telemetry(telemetry)
        teleSpace()
//        scoringSubsystem.telemetry(telemetry)
//        teleSpace()
//        TargetLock.telemetry(telemetry)
//        teleSpace()
        telemetry.update()
    }

    private fun teleSpace() {
        telemetry.addData("---------", "---------")
    }

    private fun moveOnInit(
        myOpMode: LinearOpMode,
        armSubsystem: ArmSubsystem,
        scoringSubsystem: ScoringSubsystem
    ) {
        if ((!myOpMode.gamepad1.atRest() || !myOpMode.gamepad2.atRest()) && !hasMovedOnInit) {
            scoringSubsystem.setup()
            hasMovedOnInit = true
        }
    }
}

