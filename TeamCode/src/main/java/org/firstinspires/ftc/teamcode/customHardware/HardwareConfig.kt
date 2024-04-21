//import
package org.firstinspires.ftc.teamcode.customHardware

import android.os.Environment
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.UtilClass.FileWriterFTC.setUpFile
import org.firstinspires.ftc.teamcode.UtilClass.varConfigurations.LoopTime
import org.firstinspires.ftc.teamcode.UtilClass.varConfigurations.varConfig
import org.firstinspires.ftc.teamcode.customHardware.sensors.BeamBreakSensor
import org.firstinspires.ftc.teamcode.customHardware.sensors.LEDIndicator
import org.firstinspires.ftc.teamcode.customHardware.servos.AxonServo
import org.firstinspires.ftc.teamcode.extensions.BlinkExtensions.currentColor
import org.firstinspires.ftc.teamcode.extensions.BlinkExtensions.initLights
import org.firstinspires.ftc.teamcode.extensions.PoseExtensions.toPoint
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.currentVoltage
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.getEncoderPosition
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.initDigiChan
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.initPotent
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.initVSensor
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.ledIND
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.lowVoltage
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.telemetry
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.telemetryPotent
import org.firstinspires.ftc.teamcode.humanInput.Drivers
import org.firstinspires.ftc.teamcode.humanInput.Drivers.bindDriverButtons
import org.firstinspires.ftc.teamcode.humanInput.Drivers.fieldCentric
import org.firstinspires.ftc.teamcode.humanInput.Drivers.switchProfile
import org.firstinspires.ftc.teamcode.humanInput.Operator.bindOtherButtons
import org.firstinspires.ftc.teamcode.rr.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.EndgameSubsystem
import org.firstinspires.ftc.teamcode.subsystems.ExtendoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem
import org.firstinspires.ftc.teamcode.subsystems.avoidance.AvoidanceSubsystem
import org.firstinspires.ftc.teamcode.subsystems.loopTime.LoopTimeController
import org.firstinspires.ftc.teamcode.subsystems.loopTime.PeriodicLoopTimeObject
import java.io.FileWriter


open class HardwareConfig() {

    constructor(opMode: LinearOpMode, ahwMap: HardwareMap, auto: Boolean) : this() {
        myOpMode = opMode
        this.initRobot(ahwMap, auto)
    }

    companion object {
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

        var useFileWriter: Boolean = varConfig.useFileWriter
        var multipleDrivers: Boolean = varConfig.multipleDrivers
        const val cam1_N = "Webcam 1"
        const val cam2_N = "Webcam 2"
        lateinit var lights: RevBlinkinLedDriver
        var loops = 0.0
        var LPS = 0.0
        var refreshRate = 0.0
        var rrPS = 0.0
        var currentTime = 0.0
        private var pastRefreshRate = refreshRate
        private var pastSecondLoops = 0.0
        private var pastTimeRR = 0.0
        var lastTimeOpen = 0.0
        private var pastUseLoopTime: Boolean = LoopTime.useLoopTime
        lateinit var green1: DigitalChannel
        lateinit var green2: DigitalChannel
        lateinit var green3: DigitalChannel
        lateinit var green4: DigitalChannel
        lateinit var red1: DigitalChannel
        lateinit var red2: DigitalChannel
        lateinit var red3: DigitalChannel
        lateinit var red4: DigitalChannel
        lateinit var potentiometer: AnalogInput
        lateinit var vSensor: VoltageSensor
        lateinit var drive: MecanumDrive
        lateinit var fileWriter: FileWriter
        lateinit var loopTimeController: LoopTimeController
        private lateinit var myOpMode: LinearOpMode
        var once = false
        var correctedLPS = 5

        const val CURRENT_VERSION = "7.0.0"

        var allHubs: List<LynxModule> = ArrayList()
    }

    fun initRobot(
        ahwMap: HardwareMap,
        auto: Boolean,
    ) {
        vSensor = initVSensor(ahwMap, "Expansion Hub 2")
        lights = initLights(ahwMap, "blinkin")
        // rev potentiometer //analog
        potentiometer = initPotent(ahwMap, "potent")
        green1 = initDigiChan(ahwMap, "green1")
        green2 = initDigiChan(ahwMap, "green2")
        green3 = initDigiChan(ahwMap, "green3")
        green4 = initDigiChan(ahwMap, "green4")
        red1 = initDigiChan(ahwMap, "red1")
        red2 = initDigiChan(ahwMap, "red2")
        red3 = initDigiChan(ahwMap, "red3")
        red4 = initDigiChan(ahwMap, "red4")
        allHubs = ahwMap.getAll(LynxModule::class.java)
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }
        driveSubsystem = DriveSubsystem(ahwMap)
        clawSubsystem = ClawSubsystem(ahwMap)
        endgameSubsystem = EndgameSubsystem(ahwMap)
        extendoSubsystem = ExtendoSubsystem(ahwMap)
        localizationSubsystem = LocalizationSubsystem(ahwMap)
        avoidanceSubsystem = AvoidanceSubsystem(AvoidanceSubsystem.AvoidanceTypes.PUSH)

        drive = driveSubsystem.drive
        telemetry = MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().telemetry)
        dashboard = FtcDashboard.getInstance()

        once = false
        val file = String.format(
            "%s/FIRST/matchlogs/log.txt",
            Environment.getExternalStorageDirectory().absolutePath
        )
        fileWriter = FileWriter(file, true)
        setUpFile(fileWriter)
        val timer = ElapsedTime()
        timer.reset()
        green1.ledIND(red1, true)
        green2.ledIND(red2, true)
        green3.ledIND(red3, true)
        green4.ledIND(red4, true)
        telemetry.addData("Color", lights.currentColor())
        telemetry.addData("Version", CURRENT_VERSION)
        telemetry.addData("Voltage", "%.2f", vSensor.currentVoltage())
        if (vSensor.lowVoltage()) {
            telemetry.addData("lowBattery", "true")
        }
        if (!auto) {
            telemetry.update()
        }
        drawPackets()


        loopTimeController = LoopTimeController(
            timer, listOf(PeriodicLoopTimeObject(
                "Drive", 3
            ) { drive.updatePoseEstimate() })
        )

        var servo: AxonServo = AxonServo(ahwMap, "servo")
        servo.getEncoderPosition()
        var beamBreakSensor: BeamBreakSensor = BeamBreakSensor(ahwMap, "beamBreak")
        beamBreakSensor.isBroken()
        var led1: LEDIndicator = LEDIndicator(ahwMap, 1)
        led1.turnGreen()
    }

    //code to run all drive functions
    fun doBulk() {
//        currentTime = timer.seconds()
//        loopTimeCalculations()


//        updateDashboardVariables()
        bindDriverButtons(myOpMode, driveSubsystem, clawSubsystem, endgameSubsystem)
        bindOtherButtons(myOpMode, clawSubsystem, extendoSubsystem, driveSubsystem)
        if (multipleDrivers) {
            switchProfile(myOpMode)
        }
        driveSubsystem.driveByGamepads(
            fieldCentric,
            myOpMode
        )
        driveSubsystem.update(avoidanceSubsystem)
        endgameSubsystem.update()
        clawSubsystem.update()
        extendoSubsystem.update()
        avoidanceSubsystem.update(drive)
        localizationSubsystem.relocalize(drive)
        buildTelemetry() //makes telemetry
        lynxModules()
        if (currentTime > correctedLPS) {
            loops++
        }

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
        telemetry.addData("Drivers", Drivers.currDriver + " " + Drivers.currOther)
        vSensor.telemetry(telemetry)
        telemetry.addData("Pose: ", drive.pose.toPoint().toString())
        potentiometer.telemetryPotent(telemetry)
        driveSubsystem.telemetry(telemetry)
        avoidanceSubsystem.telemetry(telemetry)
        extendoSubsystem.telemetry(telemetry)
        teleSpace()
        telemetry.addData("Timer", "%.1f", currentTime) //shows current time
//        telemetry.addData("Loops", "%.1f", loops)
        telemetry.addData("Current LPS", "%.1f", LPS)
//        telemetry.addData("Refresh Rate", "%.1f", rrPS)
        teleSpace()
//        telemetry.addData("Color", lights.currentColor())
//        teleSpace()
        telemetry.addData("Version", CURRENT_VERSION)
        localizationSubsystem.telemetry(telemetry)

        loopTimeController.telemetry(telemetry)

        telemetry.update()
        drawPackets()
    }

    private fun drawPackets() {
        packet = TelemetryPacket()

        localizationSubsystem.draw(packet)

        avoidanceSubsystem.draw(packet, drive)

//        driveSubsystem.odometrySubsystem.draw(packet, robotRad)

        dashboard.sendTelemetryPacket(packet)
    }

    private fun teleSpace() {
        telemetry.addLine(" ")
    }

//    private fun loopTimeCalculations() {
//        if (pastSecondLoops != LoopTime.loopInterval) {
//            timer.reset()
//            loops = 0.0
//            refreshRate = 0.0
//            pastSecondLoops = LoopTime.loopInterval
//        }
//        if (LoopTime.useLoopTime != pastUseLoopTime) {
//            timer.reset()
//            loops = 0.0
//            refreshRate = 0.0
//            pastUseLoopTime = LoopTime.useLoopTime
//        }
//        LPS = loops / (currentTime - correctedLPS)
//        if (refreshRate != pastRefreshRate) {
//            rrPS = currentTime - pastTimeRR
//            pastRefreshRate = refreshRate
//            pastTimeRR = currentTime
//        }
//
//        //periodic
//
//        if (LoopTime.useLoopTime && loops % LoopTime.loopInterval == 0.0) {
//            refreshRate++
//            // Update pose estimate
//            drive.updatePoseEstimate()
//        }
//    }
}

