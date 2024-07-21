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
import org.firstinspires.ftc.teamcode.subsystems.ReLocalizationSubsystem
import org.firstinspires.ftc.teamcode.subsystems.avoidance.AvoidanceSubsystem
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.ClawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.EndgameSubsystem
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.ExtendoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.humanInput.Drivers
import org.firstinspires.ftc.teamcode.subsystems.humanInput.Drivers.bindDriverButtons
import org.firstinspires.ftc.teamcode.subsystems.humanInput.Drivers.currentFieldCentric
import org.firstinspires.ftc.teamcode.subsystems.humanInput.Drivers.switchProfile
import org.firstinspires.ftc.teamcode.subsystems.humanInput.Operators.bindOtherButtons
import org.firstinspires.ftc.teamcode.subsystems.loopTime.LoopTimeController
import org.firstinspires.ftc.teamcode.subsystems.loopTime.PeriodicLoopTimeObject
import org.firstinspires.ftc.teamcode.subsystems.loopTime.SpacedBooleanObject
import org.firstinspires.ftc.teamcode.utilClass.FileWriterFTC.setUpFile
import org.firstinspires.ftc.teamcode.utilClass.drivetrain.Drivetrain
import org.firstinspires.ftc.teamcode.utilClass.drivetrain.Drivetrain.Companion.drivetrainHasPermission
import org.firstinspires.ftc.teamcode.utilClass.objects.Permission
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.varConfig
import java.io.FileWriter


open class HardwareConfig(
    private val myOpMode: LinearOpMode,
    auto: Boolean,
    ahwMap: HardwareMap = myOpMode.hardwareMap
) {

    init {
        initRobot(ahwMap, auto)
    }

    lateinit var driveSubsystem: DriveSubsystem
    lateinit var localizerSubsystem: LocalizerSubsystem
    lateinit var clawSubsystem: ClawSubsystem
    lateinit var endgameSubsystem: EndgameSubsystem
    lateinit var extendoSubsystem: ExtendoSubsystem
    lateinit var reLocalizationSubsystem: ReLocalizationSubsystem
    lateinit var avoidanceSubsystem: AvoidanceSubsystem

    companion object {
        fun isMainDrivetrain(): Boolean {
            return CurrentDrivetrain.currentDrivetrain.name == Drivetrain.DrivetrainNames.MAIN
        }

        fun isTesterDrivetrain(): Boolean {
            return CurrentDrivetrain.currentDrivetrain.name == Drivetrain.DrivetrainNames.TESTER
        }

        lateinit var telemetry: Telemetry
        lateinit var dashboard: FtcDashboard
        lateinit var packet: TelemetryPacket
        val timer: ElapsedTime = ElapsedTime()

        lateinit var axonServo: AxonServo
        lateinit var beamBreakSensor: BeamBreakSensor

        var useFileWriter: Boolean = varConfig.useFileWriter
        const val CAM1 = "Webcam 1"
        const val CAM2 = "Webcam 2"
        lateinit var lights: RevBlinkinLedDriver
        var lastTimeOpen = 0.0

        lateinit var vSensor: VoltageSensor
        lateinit var fileWriter: FileWriter
        lateinit var loopTimeController: LoopTimeController

        //        lateinit var sensorArray: SensorArray
        var once = false

        private const val CURRENT_VERSION = "7.6.0"

        var allHubs: List<LynxModule> = ArrayList()
    }

    fun initRobot(
        ahwMap: HardwareMap,
        auto: Boolean,
        startPose: Pose2d = Pose2d(0.0, 0.0, 0.0)
    ) {
        val drivetrain = CurrentDrivetrain.currentDrivetrain

        localizerSubsystem =
            LocalizerSubsystem(ahwMap, startPose, LocalizerSubsystem.LocalizationType.PP)
        driveSubsystem = DriveSubsystem(ahwMap, localizerSubsystem)

        allHubs = ahwMap.getAll(LynxModule::class.java)
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

        if (isMainDrivetrain()) vSensor =
            initVSensor(ahwMap, "Expansion Hub 2")
        if (isTesterDrivetrain()) vSensor =
            initVSensor(ahwMap, "Control Hub")
        if (drivetrainHasPermission(Permission.LIGHTS)) lights =
            initLights(ahwMap, "blinkin")
        if (drivetrainHasPermission(Permission.CLAW)) clawSubsystem =
            ClawSubsystem(ahwMap)
        if (drivetrainHasPermission(Permission.ENDGAME)) endgameSubsystem =
            EndgameSubsystem(ahwMap)
        if (drivetrainHasPermission(Permission.EXTENDO)) extendoSubsystem =
            ExtendoSubsystem(ahwMap)
        if (drivetrainHasPermission(Permission.RELOCALIZATION)) reLocalizationSubsystem =
            ReLocalizationSubsystem(ahwMap)
        avoidanceSubsystem = AvoidanceSubsystem()

        telemetry =
            MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().telemetry)
        packet = TelemetryPacket()
        dashboard = FtcDashboard.getInstance()
        once = false

        val file = String.format(
            "%s/FIRST/matchlogs/log.txt",
            Environment.getExternalStorageDirectory().absolutePath
        )
        fileWriter = FileWriter(file, true)
        setUpFile(fileWriter)


        val loopTimePeriodics = listOf(
            PeriodicLoopTimeObject(
                "Drive", 1
            ) { localizerSubsystem.update(timer) },
        )
        val spacedObjects: List<SpacedBooleanObject> = emptyList()

        loopTimeController = LoopTimeController(
            timer, loopTimePeriodics, spacedObjects
        )

        telemetry.addData("Version", CURRENT_VERSION)
        telemetry.addData("Voltage", "%.2f", vSensor.currentVoltage())
        drivetrain.telemetry(telemetry)
        if (vSensor.lowVoltage()) {
            telemetry.addData("lowBattery", "true")
        }
        if (!auto) {
            telemetry.update()
        }
        drawPackets()

        if (drivetrainHasPermission(Permission.EXTRAS)) {
            axonServo =
                AxonServo(ahwMap, "airplaneRotation", 90.0)

            beamBreakSensor =
                BeamBreakSensor(ahwMap, "beamBreak")
        }
//        sensorArray = SensorArray()
//                sensorArray.addSensor(
//                )
    }

    //code to run all drive functions
    fun doBulk() {
        val currentAvoidanceType =
            bindDriverButtons(myOpMode, driveSubsystem, null)
        if (isMainDrivetrain()) bindOtherButtons(
            myOpMode,
            clawSubsystem,
            extendoSubsystem,
        )
        if (varConfig.multipleDrivers) {
            switchProfile(myOpMode)
        }
        driveSubsystem.driveByGamepads(
            currentFieldCentric,
            myOpMode,
        )
        driveSubsystem.update(avoidanceSubsystem, currentAvoidanceType)

        if (drivetrainHasPermission(Permission.ENDGAME)) endgameSubsystem.update()

        if (drivetrainHasPermission(Permission.CLAW)) clawSubsystem.update()

        if (drivetrainHasPermission(Permission.EXTENDO)) extendoSubsystem.update()

        if (drivetrainHasPermission(Permission.RELOCALIZATION)) reLocalizationSubsystem.relocalize(
            localizerSubsystem
        )
        buildTelemetry() //makes telemetry
        lynxModules()
        loopTimeController.update()
//        sensorArray.autoLoop(loopTimeController.loops)
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
        if (varConfig.multipleDrivers) {
            telemetry.addData(
                "Drivers",
                Drivers.currDriver.name.toString() + " " + Drivers.currOther.name.toString()
            )
        }
        if (vSensor.lowVoltage()) {
            vSensor.telemetry(telemetry)
            teleSpace()
        }
        loopTimeController.telemetry(telemetry)
        teleSpace()
        driveSubsystem.telemetry(telemetry)
        localizerSubsystem.telemetry(telemetry)
        avoidanceSubsystem.telemetry(telemetry)
        if (drivetrainHasPermission(Permission.EXTENDO)) extendoSubsystem.telemetry(telemetry)
        teleSpace()
        if (drivetrainHasPermission(Permission.RELOCALIZATION)) reLocalizationSubsystem.telemetry(
            telemetry
        )

//        sensorArray.allTelemetry(telemetry)

        if (drivetrainHasPermission(Permission.EXTRAS)) {
            axonServo.telemetry(telemetry)
            beamBreakSensor.telemetry(telemetry)
        }

        teleSpace()
        telemetry.addData("Version", CURRENT_VERSION)
        telemetry.update()
        drawPackets()
    }

    private fun drawPackets() {
        packet = TelemetryPacket()
        if (drivetrainHasPermission(Permission.RELOCALIZATION)) reLocalizationSubsystem.draw(packet)

        avoidanceSubsystem.draw(packet)
        localizerSubsystem.draw(packet)

        dashboard.sendTelemetryPacket(packet)
    }

    private fun teleSpace() {
        telemetry.addLine("")
    }
}

