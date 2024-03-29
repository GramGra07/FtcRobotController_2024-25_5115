//import
package org.firstinspires.ftc.teamcode.opModes

import android.os.Environment
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Vector2d
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
import org.firstinspires.ftc.teamcode.extensions.PoseExtensions.toPoint
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.currentVoltage
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.initDigiChan
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.initPotent
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.initVSensor
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.ledIND
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.lowVoltage
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.potentAngle
import org.firstinspires.ftc.teamcode.rr.MecanumDrive
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
        fun once(myOpMode: OpMode) {
            if (!once) {
                timer.reset()
                myOpMode.gamepad1.setLedColor(229.0, 74.0, 161.0, -1)
                myOpMode.gamepad2.setLedColor(0.0, 0.0, 0.0, -1)
                once = true
            }
        }

        lateinit var telemetry: Telemetry
        lateinit var dashboard: FtcDashboard
        lateinit var packet: TelemetryPacket
        val timer: ElapsedTime = ElapsedTime()

        lateinit var driveSubsystem: DriveSubsystem
        lateinit var clawSubsystem: ClawSubsystem
        lateinit var endgameSubsystem: EndgameSubsystem
        lateinit var extendoSubsystem: ExtendoSubsystem

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
        private lateinit var myOpMode: LinearOpMode
        var once = false

        //        lateinit var startDist: StartDist
        const val CURRENT_VERSION = "7.0.0"

        var allHubs: List<LynxModule> = ArrayList()
    }

    fun init(
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

//        CommandScheduler.getInstance().registerSubsystem(driveSubsystem)
//        CommandScheduler.getInstance().registerSubsystem(clawSubsystem)
//        CommandScheduler.getInstance().registerSubsystem(endgameSubsystem)
//        CommandScheduler.getInstance().registerSubsystem(extendoSubsystem)
//        CommandScheduler.getInstance().setDefaultCommand(driveSubsystem, DriveSubsystem.driveDefault())
//        CommandScheduler.getInstance().setDefaultCommand(clawSubsystem, ClawSubsystem.clawDefault())
//        CommandScheduler.getInstance().setDefaultCommand(endgameSubsystem, EndgameSubsystem.endGameDefault())
//        CommandScheduler.getInstance().setDefaultCommand(extendoSubsystem, ExtendoSubsystem.extendoDefault())

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
    }

    //code to run all drive functions
    fun doBulk() {
        currentTime = timer.seconds()
        loopTimeCalculations()
        bindDriverButtons(myOpMode, driveSubsystem, clawSubsystem, endgameSubsystem)
        bindOtherButtons(myOpMode, clawSubsystem, extendoSubsystem, driveSubsystem)
        if (multipleDrivers) {
            switchProfile(myOpMode)
        }
        driveSubsystem.driveByGamepads(
            fieldCentric,
            myOpMode
        ) //runs drive
//        driveSubsystem.update()
//        endgameSubsystem.update()
//        clawSubsystem.update()
//        extendoSubsystem.update()
        buildTelemetry() //makes telemetry
        lynxModules()
        loops++
    }

    private fun lynxModules() {
        for (hub in allHubs) {
            hub.clearBulkCache()
        }
    }

    private fun buildTelemetry() {
        telemetry.addData("Drivers", Drivers.currDriver + " " + Drivers.currOther)
        telemetry.addData(
            "Voltage",
            "%.1f",
            vSensor.currentVoltage()
        ) //shows current battery voltage
        if (vSensor.lowVoltage()) {
            telemetry.addData("", "We have a low battery")
        }
        telemetry.addData("Pose: ", drive.pose.toPoint().toString())
        telemetry.addData("potentiometer", "%.1f", potentiometer.potentAngle())
        driveSubsystem.telemetry(telemetry)
        extendoSubsystem.telemetry(telemetry)
        teleSpace()
        telemetry.addData("Timer", "%.1f", currentTime) //shows current time
        telemetry.addData("Loops", "%.1f", loops)
        telemetry.addData("Current LPS", "%.1f", LPS)
        telemetry.addData("Refresh Rate", "%.1f", rrPS)
        teleSpace()
        telemetry.addData("Color", lights.currentColor())
        teleSpace()
        telemetry.addData("Version", CURRENT_VERSION)

        telemetry.update()
        drawPackets()
    }

    private fun drawPackets() {
        packet = TelemetryPacket()
        val rad = driveSubsystem.avoidanceSubsystem.rad
        val roboRad = varConfig.robotRadiusAvoidance
        packet.fieldOverlay()
            .setFill("red")
            .setAlpha(0.3)
        for (field in driveSubsystem.avoidanceSubsystem.fields) {
            packet.fieldOverlay()
                .fillCircle(field.point?.y!!, field.point!!.x!!, rad)
        }
        val t = drive.pose
        val halfv: Vector2d = t.heading.vec().times(0.5 * roboRad)
        val p1: Vector2d = t.position.plus(halfv)
        val (x, y) = p1.plus(halfv)
        packet.fieldOverlay()
            .setStrokeWidth(1)
            .setFill("DeepPink")
            .setAlpha(1.0)
            .strokeCircle(t.position.x, t.position.y, roboRad).strokeLine(p1.x, p1.y, x, y)
//        val poseX = driveSubsystem!!.odometrySubsystem!!.poseX
//        val poseY = driveSubsystem!!.odometrySubsystem!!.poseY
//        val heading = driveSubsystem!!.odometrySubsystem!!.heading
//        packet.fieldOverlay()
//            .setFill("DeepPink")
//            .setAlpha(1.0)
//            .strokeCircle(poseX,poseY, rad)
//            .strokeLine(
//                poseX,
//                poseY,
//                poseX + rad/2 * cos(Math.toRadians(heading)),
//                poseY + rad/2 * sin(Math.toRadians(heading))
//            )

        dashboard.sendTelemetryPacket(packet)
    }

    private fun teleSpace() {
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
        LPS = loops / currentTime
        if (refreshRate != pastRefreshRate) {
            rrPS = currentTime - pastTimeRR
            pastRefreshRate = refreshRate
            pastTimeRR = currentTime
        }

        //periodic

        if (LoopTime.useLoopTime && loops % LoopTime.loopInterval == 0.0) {
            refreshRate++
        }
    }
}

