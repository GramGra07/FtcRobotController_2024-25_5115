package org.firstinspires.ftc.teamcode.customHardware

import android.os.Environment
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.customHardware.gamepad.CustomGamepad
import org.firstinspires.ftc.teamcode.customHardware.sensorArray.SensorArray
import org.firstinspires.ftc.teamcode.customHardware.sensors.BeamBreakSensor
import org.firstinspires.ftc.teamcode.customHardware.servos.AxonServo
import org.firstinspires.ftc.teamcode.extensions.BlinkExtensions.initLights
import org.firstinspires.ftc.teamcode.extensions.PoseExtensions.toPoint
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.currentVoltage
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.initVSensor
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.lowVoltage
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.telemetry
import org.firstinspires.ftc.teamcode.followers.rr.MecanumDrive
import org.firstinspires.ftc.teamcode.storage.CurrentDrivetrain
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem
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
    lateinit var clawSubsystem: ClawSubsystem
    lateinit var endgameSubsystem: EndgameSubsystem
    lateinit var extendoSubsystem: ExtendoSubsystem
    lateinit var localizationSubsystem: LocalizationSubsystem
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
        lateinit var drive: MecanumDrive
        lateinit var fileWriter: FileWriter
        lateinit var loopTimeController: LoopTimeController
        lateinit var sensorArray: SensorArray
        var once = false

        private const val CURRENT_VERSION = "7.6.0"

        var allHubs: List<LynxModule> = ArrayList()

        lateinit var gamepad1: CustomGamepad
        lateinit var gamepad2: CustomGamepad
    }

    fun initRobot(
        ahwMap: HardwareMap,
        auto: Boolean,
        startPose: Pose2d = Pose2d(0.0, 0.0, 0.0)
    ) {
        val drivetrain = CurrentDrivetrain.currentDrivetrain

        driveSubsystem = DriveSubsystem(ahwMap, startPose)

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
        if (drivetrainHasPermission(Permission.LOCALIZATION)) localizationSubsystem =
            LocalizationSubsystem(ahwMap)
        avoidanceSubsystem = AvoidanceSubsystem()
        drive = driveSubsystem.drive

        telemetry =
            MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().telemetry)
        dashboard = FtcDashboard.getInstance()
        once = false

        gamepad1 = CustomGamepad(myOpMode.gamepad1)
        gamepad2 = CustomGamepad(myOpMode.gamepad2)

        val file = String.format(
            "%s/FIRST/matchlogs/log.txt",
            Environment.getExternalStorageDirectory().absolutePath
        )
        fileWriter = FileWriter(file, true)
        setUpFile(fileWriter)


        val loopTimePeriodics = listOf(
            PeriodicLoopTimeObject(
                "Drive", 3
            ) { drive.updatePoseEstimate() },
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
        sensorArray = SensorArray()
//                sensorArray.addSensor(
//                )
    }

    //code to run all drive functions
    fun doBulk() {
        val currentAvoidanceType =
            bindDriverButtons(myOpMode, driveSubsystem, null, packet)
        if (isMainDrivetrain()) bindOtherButtons(
            myOpMode,
            clawSubsystem,
            extendoSubsystem,
            driveSubsystem
        )
        if (varConfig.multipleDrivers) {
            switchProfile(myOpMode)
        }
        driveSubsystem.driveByGamepads(
            currentFieldCentric,
            myOpMode,
            loopTimeController.currentTime,
        )
        driveSubsystem.update(avoidanceSubsystem, currentAvoidanceType)

        if (drivetrainHasPermission(Permission.ENDGAME)) endgameSubsystem.update()

        if (drivetrainHasPermission(Permission.CLAW)) clawSubsystem.update()

        if (drivetrainHasPermission(Permission.EXTENDO)) extendoSubsystem.update()

        if (drivetrainHasPermission(Permission.LOCALIZATION)) localizationSubsystem.relocalize(drive)
        buildTelemetry() //makes telemetry
        lynxModules()
        loopTimeController.update()
        sensorArray.autoLoop(loopTimeController.loops)

        gamepad1.update()
        gamepad2.update()
    }

    fun once(myOpMode: OpMode) {
        if (!once) {
            telemetry.clearAll()
            timer.reset()
            gamepad1.setColor(CustomGamepad.Colors.HOT_PINK)
            gamepad2.setColor(CustomGamepad.Colors.BLACK)
//            myOpMode.gamepad1.setLedColor(229.0, 74.0, 161.0, -1)
//            myOpMode.gamepad2.setLedColor(0.0, 0.0, 0.0, -1)

            packet = TelemetryPacket()
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
        telemetry.addData("Pose: ", drive.pose.toPoint().toString())
        driveSubsystem.telemetry(telemetry)
        avoidanceSubsystem.telemetry(telemetry)
        if (drivetrainHasPermission(Permission.EXTENDO)) extendoSubsystem.telemetry(telemetry)
        teleSpace()
        if (drivetrainHasPermission(Permission.LOCALIZATION)) localizationSubsystem.telemetry(
            telemetry
        )

        sensorArray.allTelemetry(telemetry)

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
        if (drivetrainHasPermission(Permission.LOCALIZATION)) localizationSubsystem.draw(packet)

        avoidanceSubsystem.draw(packet, drive)

        dashboard.sendTelemetryPacket(packet)
    }

    private fun teleSpace() {
        telemetry.addLine(" ")
    }
}

