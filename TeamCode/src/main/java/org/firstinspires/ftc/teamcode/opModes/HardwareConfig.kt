//import
package org.firstinspires.ftc.teamcode.opModes

import android.os.Environment
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
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
import org.firstinspires.ftc.teamcode.Drivers
import org.firstinspires.ftc.teamcode.Drivers.bindDriverButtons
import org.firstinspires.ftc.teamcode.Drivers.fieldCentric
import org.firstinspires.ftc.teamcode.Drivers.switchProfile
import org.firstinspires.ftc.teamcode.Operator.bindOtherButtons
import org.firstinspires.ftc.teamcode.UtilClass.FileWriterFTC.setUpFile
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.LoopTime
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.varConfig
import org.firstinspires.ftc.teamcode.extensions.BlinkExtensions.currentColor
import org.firstinspires.ftc.teamcode.extensions.BlinkExtensions.initLights
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.currentVoltage
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.initDigiChan
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.initPotent
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.initVSensor
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.ledIND
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.lowVoltage
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.potentAngle
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.EndgameSubsystem
import org.firstinspires.ftc.teamcode.subsystems.ExtendoSubsystem
import java.io.FileWriter


open class HardwareConfig() {

    constructor(opMode: LinearOpMode, ahwMap: HardwareMap, auto: Boolean) : this() {
        myOpMode = opMode
        this.init(ahwMap, auto)
    }

    companion object {
        val timer: ElapsedTime = ElapsedTime()

        var driveSubsystem: DriveSubsystem? = null
        var clawSubsystem: ClawSubsystem? = null
        var endgameSubsystem: EndgameSubsystem? = null
        var extendoSubsystem: ExtendoSubsystem? = null

        var useFileWriter: Boolean = varConfig.useFileWriter
        var multipleDrivers: Boolean = varConfig.multipleDrivers
        var statusVal = "OFFLINE"
        lateinit var lights: RevBlinkinLedDriver
        var loops = 0.0
        var LPS = 0.0
        var refreshRate = 0.0
        var rrPS = 0.0
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
        var fileWriter: FileWriter? = null
        private lateinit var myOpMode: LinearOpMode
        var once = false

        //        lateinit var startDist: StartDist
        const val CURRENT_VERSION = "7.0.0"
        fun updateStatus(status: String) {
            statusVal = status
        }
    }

    fun init(
        ahwMap: HardwareMap,
        auto: Boolean,
    ) {
        val allHubs = ahwMap.getAll(
            LynxModule::class.java
        )
        for (module in allHubs) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }
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
        if (driveSubsystem == null) {
            driveSubsystem = DriveSubsystem(ahwMap)
        }
        if (clawSubsystem == null) {
            clawSubsystem = ClawSubsystem(ahwMap)
        }
        if (endgameSubsystem == null) {
            endgameSubsystem = EndgameSubsystem(ahwMap)
        }
        if (extendoSubsystem == null) {
            extendoSubsystem = ExtendoSubsystem(ahwMap)
        }

        val telemetry: Telemetry =
            MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().telemetry)

        driveSubsystem!!.reset()

//        thisDist = 0.0
        once = false
        val file = String.format(
            "%s/FIRST/matchlogs/log.txt",
            Environment.getExternalStorageDirectory().absolutePath
        )
        fileWriter = FileWriter(file, true)
        setUpFile(fileWriter!!)
        updateStatus("Initializing")
        val timer = ElapsedTime()
        timer.reset()
        green1.ledIND(red1, true)
        green2.ledIND(red2, true)
        green3.ledIND(red3, true)
        green4.ledIND(red4, true)
        telemetry.addData("Status", "Initialized")
        telemetry.addData("Color", lights.currentColor())
        telemetry.addData("Version", CURRENT_VERSION)
        telemetry.addData("Voltage", "%.2f", vSensor.currentVoltage())
        if (vSensor.lowVoltage()) {
            telemetry.addData("lowBattery", "true")
        }
        if (!auto) {
            telemetry.update()
        }
    }

    //code to run all drive functions
    fun doBulk() {
        once(myOpMode) //runs once
        periodically() //runs every loop
        loopTimeCalculations()
        bindDriverButtons(myOpMode, driveSubsystem!!, clawSubsystem!!, endgameSubsystem!!)
        bindOtherButtons(myOpMode, clawSubsystem!!, extendoSubsystem!!, driveSubsystem!!)
        if (multipleDrivers) {
            switchProfile(myOpMode)
        }
        driveSubsystem!!.driveByGamepads(fieldCentric, myOpMode) //runs drive
//        drive(fieldCentric)
        update() //sets power to power variables
        buildTelemetry() //makes telemetry
        loops++
    }

    fun once(myOpMode: OpMode) {
        if (!once) {
            timer.reset()
            val telemetry: Telemetry =
                MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().telemetry)
            // Telemetry telemetry = myOpMode.telemetry;
            telemetry.clearAll()
            updateStatus("Running")
            myOpMode.gamepad1.setLedColor(229.0, 74.0, 161.0, -1)
            myOpMode.gamepad2.setLedColor(0.0, 0.0, 0.0, -1)
            once = true
        }
    }

    private fun periodically() {
        if (LoopTime.useLoopTime && loops % LoopTime.loopInterval == 0.0) {
            refreshRate++
        }
    }

    fun update() {
        driveSubsystem!!.update()
        endgameSubsystem!!.update()
        clawSubsystem!!.update()
        extendoSubsystem!!.update()
    }

    private fun buildTelemetry() {
        val telemetry: Telemetry =
            MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().telemetry)
        telemetry.addData("Drivers", Drivers.currDriver + " " + Drivers.currOther)
        telemetry.addData(
            "Voltage",
            "%.1f",
            vSensor.currentVoltage()
        ) //shows current battery voltage
        if (vSensor.lowVoltage()) {
            telemetry.addData("", "We have a low battery")
        }
        //        telemetry.addData("Speed",drive.getWheelVelocities()[0])
        // get potent with extension
        telemetry.addData("potentiometer", "%.1f", potentiometer.potentAngle())
        driveSubsystem!!.telemetry(telemetry)
        extendoSubsystem!!.telemetry(telemetry)
        teleSpace()
        telemetry.addData("Timer", "%.1f", timer.seconds()) //shows current time
        telemetry.addData("Loops", "%.1f", loops)
        telemetry.addData("Current LPS", "%.1f", LPS)
        telemetry.addData("Refresh Rate", "%.1f", rrPS)
        teleSpace()
        telemetry.addData("Color", lights.currentColor())
        telemetry.addData("Status", statusVal) //shows current status
        teleSpace()
        telemetry.addData("Version", CURRENT_VERSION)
        telemetry.update()
    }

    private fun teleSpace() {
        val telemetry: Telemetry =
            MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().telemetry)
        //Telemetry telemetry = myOpMode.telemetry;
        telemetry.addLine(" ")
    }

    private fun loopTimeCalculations() {
        if (pastSecondLoops != LoopTime.loopInterval) {
            timer.reset()
            loops = 0.0
            refreshRate = 0.0
            pastSecondLoops = LoopTime.loopInterval
        }
        if (LoopTime.useLoopTime != pastUseLoopTime) {
            timer.reset()
            loops = 0.0
            refreshRate = 0.0
            pastUseLoopTime = LoopTime.useLoopTime
        }
        LPS = loops / timer.seconds()
        if (refreshRate != pastRefreshRate) {
            rrPS = timer.seconds() - pastTimeRR
            pastRefreshRate = refreshRate
            pastTimeRR = timer.seconds()
        }
    }


}

