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
import org.firstinspires.ftc.teamcode.followers.roadRunner.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.DriverAid
import org.firstinspires.ftc.teamcode.subsystems.LocalizerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.ArmSubsystem
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.ScoringSubsystem
import org.firstinspires.ftc.teamcode.subsystems.humanInput.Drivers.bindDriverButtons
import org.firstinspires.ftc.teamcode.subsystems.humanInput.Drivers.currentFieldCentric
import org.firstinspires.ftc.teamcode.subsystems.humanInput.Operators.bindOtherButtons
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
    lateinit var mecanumDrive: MecanumDrive

    lateinit var armSubsystem: ArmSubsystem
    lateinit var scoringSubsystem: ScoringSubsystem
    lateinit var driverAid: DriverAid
//    lateinit var reLocalizationSubsystem: ReLocalizationSubsystem

//    lateinit var brush: BrushlandRoboticsSensor
//    lateinit var beamBreakSensor: BeamBreakSensor

    companion object {
        lateinit var telemetry: Telemetry
        lateinit var dashboard: FtcDashboard
        val timer: ElapsedTime = ElapsedTime()

        const val CAM1 = "Webcam 1"

        lateinit var loopTimeController: LoopTimeController

        var once = false
        var canMove = true

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
                PoseStorage.currentPose
            )

        driveSubsystem = DriveSubsystem(ahwMap, localizerSubsystem, auto)

        allHubs = ahwMap.getAll(LynxModule::class.java)
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }

        armSubsystem =
            ArmSubsystem(ahwMap, auto)
        scoringSubsystem =
            ScoringSubsystem(ahwMap, auto, armSubsystem)

        driverAid = DriverAid(scoringSubsystem, armSubsystem, localizerSubsystem)
//        reLocalizationSubsystem =
//            ReLocalizationSubsystem(ahwMap)

        telemetry = myOpMode.telemetry

        dashboard = FtcDashboard.getInstance()
        once = false
        localizerSubsystem.draw(dashboard)
        if (!auto) {
            canMove = false
            mecanumDrive = localizerSubsystem.drive
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

        driveSubsystem.driveByGamepads(
            currentFieldCentric,
            myOpMode,
        )
        driveSubsystem.update()

//        localizerSubsystem.update(timer)

        scoringSubsystem.update()
        if (hasMovedOnInit) {
            armSubsystem.update(
                -myOpMode.gamepad2.right_stick_y.toDouble(),
                -myOpMode.gamepad2.left_stick_y.toDouble()
            )
        }


        driverAid.update()

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
            armSubsystem.findOffset()
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
        driveSubsystem.telemetry(telemetry, false)
        teleSpace()
//        reLocalizationSubsystem.telemetry(telemetry)
//        teleSpace()
//        localizerSubsystem.telemetry(telemetry)
//        teleSpace()
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
        if ((!myOpMode.gamepad1.atRest() || !myOpMode.gamepad2.atRest()) && !hasMovedOnInit
        ) {
            scoringSubsystem.setup()
            hasMovedOnInit = true

        }
    }
}

