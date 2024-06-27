package org.firstinspires.ftc.teamcode.customHardware

import android.os.Environment
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.google.blocks.ftcrobotcontroller.util.CurrentGame
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.checkerframework.checker.units.qual.Current
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.customHardware.sensorArray.SensorArray
import org.firstinspires.ftc.teamcode.customHardware.sensors.BeamBreakSensor
import org.firstinspires.ftc.teamcode.customHardware.servos.AxonServo
import org.firstinspires.ftc.teamcode.extensions.BlinkExtensions.initLights
import org.firstinspires.ftc.teamcode.extensions.PoseExtensions.toPoint
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.currentVoltage
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.initVSensor
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.lowVoltage
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.telemetry
import org.firstinspires.ftc.teamcode.rr.MecanumDrive
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
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.varConfig
import java.io.FileWriter


open class HardwareConfig() {

    constructor(opMode: LinearOpMode, ahwMap: HardwareMap, auto: Boolean) : this() {
        myOpMode = opMode
        this.initRobot(ahwMap, auto)
    }

    companion object {
        fun isMainDrivetrain():Boolean{
            return CurrentDrivetrain.currentDrivetrain.name == Drivetrain.DrivetrainNames.MAIN
        }
        fun isTesterDrivetrain():Boolean{
            return CurrentDrivetrain.currentDrivetrain.name == Drivetrain.DrivetrainNames.TESTER
        }
        lateinit var telemetry: Telemetry
        lateinit var dashboard: FtcDashboard
        lateinit var packet: TelemetryPacket
        val timer: ElapsedTime = ElapsedTime()

        lateinit var driveSubsystem: DriveSubsystem
        lateinit var clawSubsystem: ClawSubsystem
        lateinit var endgameSubsystem: EndgameSubsystem
        lateinit var extendoSubsystem: ExtendoSubsystem
        lateinit var localizationSubsystem: LocalizationSubsystem
        lateinit var avoidanceSubsystem: AvoidanceSubsystem


        private lateinit var motorFrontLeft: DcMotorEx
        private lateinit var motorBackLeft: DcMotorEx
        private lateinit var motorFrontRight: DcMotorEx
        private lateinit var motorBackRight: DcMotorEx

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
        private lateinit var myOpMode: LinearOpMode
        var once = false

        private const val CURRENT_VERSION = "7.5.0"

        var allHubs: List<LynxModule> = ArrayList()
    }

    fun initRobot(
        ahwMap: HardwareMap,
        auto: Boolean,
    ) {
        val drivetrain = CurrentDrivetrain.currentDrivetrain

        driveSubsystem = DriveSubsystem(ahwMap)

        allHubs = ahwMap.getAll(LynxModule::class.java)
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

        if (isMainDrivetrain()) vSensor =
            initVSensor(ahwMap, "Expansion Hub 2")
        if (isTesterDrivetrain()) vSensor =
            initVSensor(ahwMap, "Control Hub")
        if (isMainDrivetrain()) lights =
            initLights(ahwMap, "blinkin")
        if (isMainDrivetrain()) clawSubsystem =
            ClawSubsystem(ahwMap)
        if (isMainDrivetrain()) endgameSubsystem =
            EndgameSubsystem(ahwMap)
        if (isMainDrivetrain()) extendoSubsystem =
            ExtendoSubsystem(ahwMap)
        if (isMainDrivetrain()) localizationSubsystem =
            LocalizationSubsystem(ahwMap)
        avoidanceSubsystem = AvoidanceSubsystem()
        drive = driveSubsystem.drive

        telemetry =
            MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().telemetry)
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

        if (isMainDrivetrain()) axonServo =
            AxonServo(ahwMap, "airplaneRotation", 90.0)

        if (isMainDrivetrain()) beamBreakSensor =
            BeamBreakSensor(ahwMap, "beamBreak")
//                sensorArray = SensorArray()
//                sensorArray.addSensor(
//                    Pair(
//                        "axon", Sensor(
//                            SensorType.ENC,
//                            { axonServo = AxonServo(ahwMap, "airplaneRotation", 90.0) },
//                            1
//                        )
//                    )
//                )
    }

    //code to run all drive functions
    fun doBulk() {
        val drivetrain = CurrentDrivetrain.currentDrivetrain
        val currentAvoidanceType =
            bindDriverButtons(myOpMode, driveSubsystem, null)
        if (isMainDrivetrain()) bindOtherButtons(myOpMode, clawSubsystem, extendoSubsystem, driveSubsystem)
        if (varConfig.multipleDrivers) {
            switchProfile(myOpMode)
        }
        driveSubsystem.driveByGamepads(
            currentFieldCentric,
            myOpMode,
            loopTimeController.currentTime,
        )
        driveSubsystem.update(avoidanceSubsystem, currentAvoidanceType)

        if (isMainDrivetrain()) endgameSubsystem.update()

        if (isMainDrivetrain()) clawSubsystem.update()

        if (isMainDrivetrain()) extendoSubsystem.update()

        if (isMainDrivetrain()) localizationSubsystem.relocalize(
            drive
        )
        buildTelemetry() //makes telemetry
        lynxModules()
        loopTimeController.update()
    }

    fun once(myOpMode: OpMode) {
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
        telemetry.addData("Pose: ", drive.pose.toPoint().toString())
        driveSubsystem.telemetry(telemetry)
        avoidanceSubsystem.telemetry(telemetry)
        if (isMainDrivetrain())extendoSubsystem.telemetry(telemetry)
        teleSpace()
        if (isMainDrivetrain())localizationSubsystem.telemetry(telemetry)

        if (isMainDrivetrain())axonServo.telemetry(telemetry)
        if (isMainDrivetrain())beamBreakSensor.telemetry(telemetry)

        teleSpace()
        telemetry.addData("Version", CURRENT_VERSION)
        telemetry.update()
        drawPackets()
    }

    private fun drawPackets() {
        packet = TelemetryPacket()

        if (isMainDrivetrain())localizationSubsystem.draw(packet)

        avoidanceSubsystem.draw(packet, drive)

        dashboard.sendTelemetryPacket(packet)
    }

    private fun teleSpace() {
        telemetry.addLine(" ")
    }
}

