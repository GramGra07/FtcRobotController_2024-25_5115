package org.firstinspires.ftc.teamcode.customHardware

import android.os.Environment
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.customHardware.camera.camUtil.CameraUtilities
import org.firstinspires.ftc.teamcode.customHardware.loopTime.PeriodicLoopTimeObject
import org.firstinspires.ftc.teamcode.customHardware.loopTime.SpacedBooleanObject
import org.firstinspires.ftc.teamcode.customHardware.sensorArray.SensorArray
import org.firstinspires.ftc.teamcode.customHardware.sensors.BeamBreakSensor
import org.firstinspires.ftc.teamcode.customHardware.servos.AxonServo
import org.firstinspires.ftc.teamcode.extensions.BlinkExtensions.initLights
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.currentVoltage
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.initVSensor
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.lowVoltage
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.telemetry
import org.firstinspires.ftc.teamcode.storage.CurrentDrivetrain
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.LocalizerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.LocalizerSubsystem.LocalizationType
import org.firstinspires.ftc.teamcode.subsystems.ReLocalizationSubsystem
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.FastIntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.LiftSubsystem
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.ScoringSubsystem
import org.firstinspires.ftc.teamcode.subsystems.humanInput.Drivers
import org.firstinspires.ftc.teamcode.subsystems.humanInput.Drivers.bindDriverButtons
import org.firstinspires.ftc.teamcode.subsystems.humanInput.Drivers.currentFieldCentric
import org.firstinspires.ftc.teamcode.subsystems.humanInput.Drivers.switchProfile
import org.firstinspires.ftc.teamcode.subsystems.humanInput.Operators.bindOtherButtons
import org.firstinspires.ftc.teamcode.subsystems.loopTime.LoopTimeController
import org.firstinspires.ftc.teamcode.utilClass.FileWriterFTC.setUpFile
import org.firstinspires.ftc.teamcode.utilClass.drawer.Drawing
import org.firstinspires.ftc.teamcode.utilClass.drivetrain.Drivetrain
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.VarConfig
import java.io.FileWriter


open class HardwareConfig(
    private val myOpMode: LinearOpMode,
    auto: Boolean,
    ahwMap: HardwareMap = myOpMode.hardwareMap
) {

    init {
        initRobot(ahwMap, auto)
    }

    //    lateinit var sparkFunOTOS: SparkFunOTOS
    lateinit var driveSubsystem: DriveSubsystem
    lateinit var localizerSubsystem: LocalizerSubsystem
    lateinit var fastIntakeSubsystem: FastIntakeSubsystem
    lateinit var liftSubsystem: LiftSubsystem
    lateinit var scoringSubsystem: ScoringSubsystem
    lateinit var reLocalizationSubsystem: ReLocalizationSubsystem

    companion object {
        //        fun isMainDrivetrain(): Boolean {
//            return CurrentDrivetrain.currentDrivetrain.name == Drivetrain.DrivetrainNames.MAIN
//        }
//
        fun isOLDDrivetrain(): Boolean {
            return CurrentDrivetrain.currentDrivetrain.name == Drivetrain.DrivetrainNames.OLD
        }

        lateinit var telemetry: Telemetry
        lateinit var dashboard: FtcDashboard
        lateinit var packet: TelemetryPacket
        val timer: ElapsedTime = ElapsedTime()

        lateinit var axonServo: AxonServo
        lateinit var beamBreakSensor: BeamBreakSensor

        var useFileWriter: Boolean = VarConfig.useFileWriter
        const val CAM1 = "Webcam 1"
        const val CAM2 = "Webcam 2"
        lateinit var lights: RevBlinkinLedDriver
        var lastTimeOpen = 0.0

        lateinit var vSensor: VoltageSensor
        lateinit var fileWriter: FileWriter
        lateinit var loopTimeController: LoopTimeController

        lateinit var sensorArray: SensorArray
        var once = false

        private const val CURRENT_VERSION = "8.0.0"

        var allHubs: List<LynxModule> = ArrayList()
    }

    fun initRobot(
        ahwMap: HardwareMap,
        auto: Boolean,
        startPose: Pose2d = Pose2d(0.0, 0.0, -Math.PI / 2)
    ) {
        val drivetrain = CurrentDrivetrain.currentDrivetrain

        localizerSubsystem =
            LocalizerSubsystem(ahwMap, startPose, LocalizationType.PPOTOS)
        driveSubsystem = DriveSubsystem(ahwMap, localizerSubsystem)

        allHubs = ahwMap.getAll(LynxModule::class.java)
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

        vSensor =
            initVSensor(ahwMap, "Expansion Hub 2")
        lights =
            initLights(ahwMap, "blinkin")
        fastIntakeSubsystem =
            FastIntakeSubsystem(ahwMap)
        liftSubsystem =
            LiftSubsystem(ahwMap)
        scoringSubsystem =
            ScoringSubsystem(ahwMap)

        reLocalizationSubsystem =
            ReLocalizationSubsystem(ahwMap)

        telemetry =
            MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().telemetry)
        packet = TelemetryPacket()
        dashboard = FtcDashboard.getInstance()
        once = false

        if (useFileWriter) {
            val file = String.format(
                "%s/FIRST/matchlogs/log.txt",
                Environment.getExternalStorageDirectory().absolutePath
            )
            fileWriter = FileWriter(file, true)
            setUpFile(fileWriter)
        }

        if (!auto) {
            val loopTimePeriodics = emptyList<PeriodicLoopTimeObject>()
            val spacedObjects: List<SpacedBooleanObject> = emptyList()
            loopTimeController = LoopTimeController(
                timer, loopTimePeriodics, spacedObjects
            )
        }

        telemetry.addData("Version", CURRENT_VERSION)
        telemetry.addData("Voltage", "%.2f", vSensor.currentVoltage())
        drivetrain.telemetry(telemetry)
        if (vSensor.lowVoltage())
            telemetry.addData("lowBattery", "true")

        if (!auto)
            telemetry.update()

        if (!loopTimeController.loopSaver) Drawing.drawAll(packet, dashboard, localizerSubsystem)
    }

    //code to run all drive functions
    fun doBulk() {
        localizerSubsystem.update(timer)
        bindDriverButtons(myOpMode, driveSubsystem, liftSubsystem)
        bindOtherButtons(
            myOpMode,
            fastIntakeSubsystem,
            scoringSubsystem
        )
        if (VarConfig.multipleDrivers) {
            switchProfile(myOpMode)
        }
        driveSubsystem.driveByGamepads(
            currentFieldCentric,
            myOpMode,
        )
        driveSubsystem.update()

        fastIntakeSubsystem.update(loopTimeController)

        scoringSubsystem.update()

        liftSubsystem.update()

//        loopTimeController.every(3) {
         reLocalizationSubsystem.relocalize(
            localizerSubsystem
        )
//        }

        buildTelemetry() //makes telemetry
        lynxModules()
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
        if (loopTimeController.loopSaver)
            CameraUtilities.stopCameraStream()
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
        loopTimeController.telemetry(telemetry)
        teleSpace()
        driveSubsystem.telemetry(telemetry)
        teleSpace()
        localizerSubsystem.telemetry(telemetry)
        teleSpace()
        fastIntakeSubsystem.telemetry(telemetry)
        teleSpace()
        scoringSubsystem.telemetry(telemetry)
        teleSpace()
        liftSubsystem.telemetry(telemetry)
        teleSpace()

        if (!loopTimeController.loopSaver) {
            reLocalizationSubsystem.telemetry(
                telemetry
            )
            teleSpace()
        }

//        beamBreakSensor.telemetry(telemetry)
        teleSpace()


        telemetry.addData("Version", CURRENT_VERSION)
        telemetry.update()
        if (!loopTimeController.loopSaver) Drawing.drawAll(packet, dashboard, localizerSubsystem)
    }

    private fun teleSpace() {
        telemetry.addData("---------", "---------")
    }
}

