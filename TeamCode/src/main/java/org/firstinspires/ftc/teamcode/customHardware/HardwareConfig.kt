package org.firstinspires.ftc.teamcode.customHardware

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.customHardware.camera.camUtil.CameraUtilities.initializeProcessor
import org.firstinspires.ftc.teamcode.customHardware.camera.camUtil.CameraUtilities.stopCameraStream
import org.firstinspires.ftc.teamcode.customHardware.camera.camUtil.PROCESSORS
import org.firstinspires.ftc.teamcode.customHardware.loopTime.LoopTimeController
import org.firstinspires.ftc.teamcode.customHardware.loopTime.LoopTimeController.Companion.every
import org.firstinspires.ftc.teamcode.extensions.BlinkExtensions.initLights
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.currentVoltage
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.initVSensor
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.lowVoltage
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.telemetry
import org.firstinspires.ftc.teamcode.storage.CurrentDrivetrain
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.LocalizerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.ReLocalizationSubsystem
import org.firstinspires.ftc.teamcode.subsystems.humanInput.Drivers
import org.firstinspires.ftc.teamcode.subsystems.humanInput.Drivers.bindDriverButtons
import org.firstinspires.ftc.teamcode.subsystems.humanInput.Drivers.currentFieldCentric
import org.firstinspires.ftc.teamcode.subsystems.humanInput.Drivers.switchProfile
import org.firstinspires.ftc.teamcode.subsystems.humanInput.Operators.bindOtherButtons
import org.firstinspires.ftc.teamcode.utilClass.drivetrain.Drivetrain
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.VarConfig
import org.firstinspires.ftc.teamcode.vision.TargetLock

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

    //    lateinit var fastIntakeSubsystem: FastIntakeSubsystem
//    lateinit var liftSubsystem: LiftSubsystem
//    lateinit var scoringSubsystem: ScoringSubsystem
    lateinit var reLocalizationSubsystem: ReLocalizationSubsystem


    companion object {
        val dt = CurrentDrivetrain.currentDrivetrain

        fun isOLDDrivetrain(): Boolean {
            return dt.name == Drivetrain.DrivetrainNames.OLD
        }

        lateinit var telemetry: Telemetry
        lateinit var dashboard: FtcDashboard
        val timer: ElapsedTime = ElapsedTime()

        const val CAM1 = "Webcam 1"
        const val CAM2 = "Webcam 2"
        lateinit var lights: RevBlinkinLedDriver
        var lastTimeOpen = 0.0

        lateinit var vSensor: VoltageSensor
        lateinit var loopTimeController: LoopTimeController

        var once = false

        private const val CURRENT_VERSION = "8.1.0"

        var allHubs: List<LynxModule> = ArrayList()
    }

    fun initRobot(
        ahwMap: HardwareMap,
        auto: Boolean,
        startLocation: StartLocation
    ) {
        localizerSubsystem =
            LocalizerSubsystem(
                ahwMap,
                startLocation.startPose,
                LocalizerSubsystem.LocalizerType.PEDRO
            )
        driveSubsystem = DriveSubsystem(ahwMap, localizerSubsystem, dt)

        allHubs = ahwMap.getAll(LynxModule::class.java)
        for (hub in allHubs) {
            hub.setConstant(15026849)
            hub.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }

//        vSensor =
//            initVSensor(ahwMap, "Expansion Hub 2")
        vSensor =
            initVSensor(ahwMap, "Control Hub")
        lights =
            initLights(ahwMap, "blinkin")
//        fastIntakeSubsystem =
//            FastIntakeSubsystem(ahwMap)
//        liftSubsystem =
//            LiftSubsystem(ahwMap)
//        scoringSubsystem =
//            ScoringSubsystem(ahwMap)

        reLocalizationSubsystem =
            ReLocalizationSubsystem(ahwMap)

        telemetry = myOpMode.telemetry
//            MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().telemetry)
        dashboard = FtcDashboard.getInstance()
        once = false

        initializeProcessor(startLocation.alliance, PROCESSORS.TARGET_LOCK, ahwMap, CAM1, true)

        if (!auto) {
            loopTimeController = LoopTimeController(timer)
        }

        telemetry.addData("Version", CURRENT_VERSION)
        telemetry.addData("Voltage", "%.2f", vSensor.currentVoltage())
        dt.telemetry(telemetry)
        if (vSensor.lowVoltage())
            telemetry.addData("lowBattery", "true")

        if (!auto)
            telemetry.update()

        localizerSubsystem.draw(dashboard)
    }

    //code to run all drive functions
    fun doBulk() {
        bindDriverButtons(
            myOpMode, driveSubsystem,
//            liftSubsystem,
        )
        bindOtherButtons(
            myOpMode,
//            fastIntakeSubsystem,
//            scoringSubsystem
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

//        fastIntakeSubsystem.update(loopTimeController)
//
//        scoringSubsystem.update()
//
//        liftSubsystem.update()

        loopTimeController.every(if (VarConfig.loopSaver) 30 else 10) {
            reLocalizationSubsystem.update(localizerSubsystem, VarConfig.relocalize)
        }

        loopTimeController.every(if (VarConfig.loopSaver) 30 else 10) {
            buildTelemetry() //makes telemetry
        }

//        if (!loopTimeController.loopSaver) {
//            startCameraStream()
//        } else {
        stopCameraStream()
//        }

        localizerSubsystem.draw(dashboard)

        loopTimeController.update()
    }

    fun once() {
        if (!once) {
            telemetry.clearAll()
            timer.reset()
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
        if (vSensor.lowVoltage()) {
            vSensor.telemetry(telemetry)
            teleSpace()
        }
        driveSubsystem.telemetry(telemetry, false)
        teleSpace()
        reLocalizationSubsystem.telemetry(telemetry)
        teleSpace()
        localizerSubsystem.telemetry(telemetry)
        teleSpace()
//            fastIntakeSubsystem.telemetry(telemetry)
//            teleSpace()
//            scoringSubsystem.telemetry(telemetry)
//            teleSpace()
//            liftSubsystem.telemetry(telemetry)
//            teleSpace()

        TargetLock.telemetry(telemetry)
        teleSpace()
        telemetry.addData("Version", CURRENT_VERSION)
        telemetry.update()
    }

    private fun teleSpace() {
        telemetry.addData("---------", "---------")
    }
}

