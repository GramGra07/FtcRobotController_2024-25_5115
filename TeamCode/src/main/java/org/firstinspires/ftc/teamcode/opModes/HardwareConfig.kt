//import
package org.firstinspires.ftc.teamcode.opModes

import android.os.Environment
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.controller.PIDFController
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Drivers
import org.firstinspires.ftc.teamcode.Drivers.bindDriverButtons
import org.firstinspires.ftc.teamcode.Drivers.fieldCentric
import org.firstinspires.ftc.teamcode.Drivers.switchProfile
import org.firstinspires.ftc.teamcode.Enums.StartDist
import org.firstinspires.ftc.teamcode.Operator.bindOtherButtons
import org.firstinspires.ftc.teamcode.UtilClass.FileWriterFTC.setUpFile
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.LoopTime
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.varConfig
import org.firstinspires.ftc.teamcode.extensions.BlinkExtensions.currentColor
import org.firstinspires.ftc.teamcode.extensions.BlinkExtensions.initLights
import org.firstinspires.ftc.teamcode.extensions.MotorExtensions.initMotor
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.currentVoltage
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.initDigiChan
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.initPotent
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.initVSensor
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.ledIND
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.lowVoltage
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.potentAngle
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.initServo
import org.firstinspires.ftc.teamcode.opModes.DistanceStorage.Companion.totalDist
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.AutoHardware
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.EndgameSubsystem
import org.firstinspires.ftc.teamcode.subsystems.ExtendoSubsystem
import java.io.FileWriter

open class HardwareConfig() {
    companion object {

        lateinit var driveSubsystem: DriveSubsystem
        lateinit var clawSubsystem: ClawSubsystem
        lateinit var endgameSubsystem: EndgameSubsystem
        lateinit var extendoSubsystem: ExtendoSubsystem

        var useFileWriter: Boolean = varConfig.useFileWriter
        var multipleDrivers: Boolean = varConfig.multipleDrivers
        var statusVal = "OFFLINE"

        //        lateinit var claw1: Servo
//        lateinit var claw2: Servo
//        lateinit var flipServo: Servo
//        lateinit var airplaneServo: Servo

        //    lateinit var motorFrontLeft: DcMotor
//    lateinit var motorBackLeft: DcMotor
//    lateinit var motorFrontRight: DcMotor
//    lateinit var motorBackRight: DcMotor
//        lateinit var motorLift: DcMotor
//        lateinit var motorExtension: DcMotor
//        lateinit var motorRotation: DcMotor
        lateinit var lights: RevBlinkinLedDriver
        var slowModeIsOn = false
        var reversed = false
//        var liftPower = 0.0
//        var extensionPower = 0.0
//        var rotationPower = 0.0
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
        var thisDist = 0.0
        val timer: ElapsedTime = ElapsedTime()
        var fileWriter: FileWriter? = null
        private lateinit var myOpMode: LinearOpMode
        var once = false
//        var extensionPIDF = PIDFController(0.0, 0.0, 0.0, 0.0)
//        var rotationPIDF = PIDFController(0.0, 0.0, 0.0, 0.0)
        lateinit var startDist: StartDist
        val currentVersion = "6.0.0"

        //init
        fun init(
            ahwMap: HardwareMap,
            auto: Boolean,
//            driveSubsystem: DriveSubsystem,
//            clawSubsystem: ClawSubsystem,
//            endgameSubsystem: EndgameSubsystem,
//            extendoSubsystem: ExtendoSubsystem
        ) {

            this.driveSubsystem = DriveSubsystem(ahwMap)
            this.clawSubsystem = ClawSubsystem(ahwMap)
            this.endgameSubsystem = EndgameSubsystem(ahwMap)
            this.extendoSubsystem = ExtendoSubsystem(ahwMap)

            val telemetry: Telemetry =
                MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().telemetry)

            driveSubsystem.reset()

//        thisDist = 0.0
            once = false
            val file = String.format(
                "%s/FIRST/matchlogs/log.txt",
                Environment.getExternalStorageDirectory().absolutePath
            )
            fileWriter = FileWriter(file, true)
            setUpFile(fileWriter!!)
            updateStatus("Initializing")
//            drive = MecanumDrive(ahwMap)
//            drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
//            drive.poseEstimate = PoseStorage.currentPose
            val timer = ElapsedTime() //declaring the runtime variable
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
            // Declare our motors
//            motorFrontLeft =
//                initMotor(ahwMap, "motorFrontLeft", DcMotor.RunMode.RUN_WITHOUT_ENCODER)
//            motorBackLeft = initMotor(
//                ahwMap,
//                "motorBackLeft",
//                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
//                DcMotorSimple.Direction.REVERSE
//            )
//            motorFrontRight =
//                initMotor(ahwMap, "motorFrontRight", DcMotor.RunMode.RUN_WITHOUT_ENCODER)
//            motorBackRight =
//                initMotor(ahwMap, "motorBackRight", DcMotor.RunMode.RUN_WITHOUT_ENCODER)
//            motorLift = initMotor(
//                ahwMap,
//                "lift",
//                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
//                DcMotorSimple.Direction.REVERSE
//            )
//            motorExtension = initMotor(ahwMap, "slideMotor", DcMotor.RunMode.RUN_USING_ENCODER)
//            motorRotation = initMotor(
//                ahwMap,
//                "flipperMotor",
//                DcMotor.RunMode.RUN_USING_ENCODER,
//                DcMotorSimple.Direction.REVERSE
//            )
//        claw1 = initServo(ahwMap, "claw1")
//        claw2 = initServo(ahwMap, "claw2")
//        flipServo = initServo(ahwMap, "flipServo")
//            airplaneServo = initServo(ahwMap, "airplaneServo")
//        ServoUtil.closeClaw(claw1)
//        ServoUtil.closeClaw(claw2)
            timer.reset()
            green1.ledIND(red1, true)
            green2.ledIND(red2, true)
            green3.ledIND(red3, true)
            green4.ledIND(red4, true)
            telemetry.addData("Status", "Initialized")
            telemetry.addData("Color", lights.currentColor())
            telemetry.addData("Version", currentVersion)
            telemetry.addData("Voltage", "%.2f", vSensor.currentVoltage())
            if (auto) {
                telemetry.addData("Random", AutoHardware.autonomousRandom)
            }
            if (vSensor.lowVoltage()) {
                telemetry.addData("lowBattery", "true")
            }
            if (!auto) {
                telemetry.update()
            }
        }

        fun updateStatus(status: String) {
            statusVal = status
        }
    }


    constructor(opMode: LinearOpMode) : this() {
        myOpMode = opMode
    }

    //code to run all drive functions
    fun doBulk() {
        once(myOpMode) //runs once
        periodically() //runs every loop
        loopTimeCalculations()
        bindDriverButtons(myOpMode, driveSubsystem, clawSubsystem, endgameSubsystem)
        bindOtherButtons(myOpMode, clawSubsystem, extendoSubsystem)
        if (multipleDrivers) {
            switchProfile(myOpMode)
        }
        driveSubsystem.driveByGamepads(fieldCentric, myOpMode) //runs drive
//        drive(fieldCentric)
        power() //sets power to power variables
        buildTelemetry() //makes telemetry
        loops++
    }

    fun once(myOpMode: OpMode) {
        if (!once) {
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
        if (LoopTime.useLoopTime) {
            if (loops % LoopTime.loopInterval == 0.0) { // happens every loopInterval, loops
                refreshRate++
            }
        }
    }

//    private var slowMult: Int = varConfig.slowMult
//    private var slowPower = 0
//    private var xControl = 0.0
//    private var yControl = 0.0
//    private var frontRightPower = 0.0
//    private var frontLeftPower = 0.0
//    private var backRightPower = 0.0
//    private var backLeftPower = 0.0
//    private var gamepadX = 0.0
//    private var gamepadY = 0.0
//    private var gamepadHypot = 0.0
//    private var controllerAngle = 0.0
//    private var robotDegree = 0.0
//    private var movementDegree = 0.0
//    var reverse = false
//    fun drive(fieldCentric: Boolean) {
//        if (fieldCentric) {
//            gamepadX =
//                myOpMode.gamepad1.left_stick_x.toDouble() //get the x val of left stick and store
//            gamepadY =
//                -myOpMode.gamepad1.left_stick_y.toDouble() //get the y val of left stick and store
//            gamepadHypot = Range.clip(Math.hypot(gamepadX, gamepadY), 0.0, 1.0) //get the
//            // hypotenuse of the x and y values,clip it to a max of 1 and store
//            controllerAngle = Math.toDegrees(
//                Math.atan2(
//                    gamepadY,
//                    gamepadX
//                )
//            ) //Get the angle of the controller stick using arc tangent
//            robotDegree = Math.toDegrees(drive.poseEstimate.heading) // change to imu
//            movementDegree =
//                controllerAngle - robotDegree //get the movement degree based on the controller vs robot angle
//            xControl =
//                Math.cos(Math.toRadians(movementDegree)) * gamepadHypot //get the x value of the movement
//            yControl =
//                Math.sin(Math.toRadians(movementDegree)) * gamepadHypot //get the y value of the movement
//            val turn: Double = -myOpMode.gamepad1.right_stick_x.toDouble()
//            frontRightPower =
//                (yControl * Math.abs(yControl) - xControl * Math.abs(xControl) + turn) / slowPower
//            backRightPower =
//                (yControl * Math.abs(yControl) + xControl * Math.abs(xControl) + turn) / slowPower
//            frontLeftPower =
//                (yControl * Math.abs(yControl) + xControl * Math.abs(xControl) - turn) / slowPower
//            backLeftPower =
//                (yControl * Math.abs(yControl) - xControl * Math.abs(xControl) - turn) / slowPower
//        } else {
////            reverse = myOpMode.gamepad1.touchpad_finger_1_x > 0.5;//0,1 left to right
////            reversed = reverse;
//            yControl = -myOpMode.gamepad1.left_stick_y.toDouble()
//            xControl = myOpMode.gamepad1.left_stick_x.toDouble()
//            if (reverse) {
//                yControl = -yControl
//                xControl = -xControl
//            }
//            val turn: Double = -myOpMode.gamepad1.right_stick_x.toDouble()
//            slowPower = if (slowModeIsOn) {
//                slowMult
//            } else {
//                1
//            }
//            frontRightPower = (yControl - xControl + turn) / slowPower
//            backRightPower = (yControl + xControl + turn) / slowPower
//            frontLeftPower = (yControl + xControl - turn) / slowPower
//            backLeftPower = (yControl - xControl - turn) / slowPower
//        }
//        drive.update()
//        updateDistTraveled(PoseStorage.currentPose, drive.poseEstimate)
//        FileWriterFTC.writeToFile(
//            fileWriter!!,
//            drive.poseEstimate.x.toInt(),
//            drive.poseEstimate.y.toInt()
//        )
//        PoseStorage.currentPose = drive.poseEstimate
//    }

//    private fun updateDistTraveled(before: Pose2d, after: Pose2d) {
//        val x = after.x - before.x
//        val y = after.y - before.y
//        val dist = sqrt(x * x + y * y)
//        thisDist += dist
//        totalDist += dist
//    }

    fun power() { // put all set power here
//        if (!IsBusy.isAutoInTeleop) {
//            motorFrontLeft.power = frontLeftPower
//            motorBackLeft.power = backLeftPower
//            motorFrontRight.power = frontRightPower
//            motorBackRight.power = backRightPower
//        }
        driveSubsystem.update()
        endgameSubsystem.update()
        clawSubsystem.update()
        extendoSubsystem.update()
//        motorExtension.power = extensionPower
//        motorRotation.power = rotationPower
    }

    fun buildTelemetry() {
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
        driveSubsystem.telemetry(telemetry)
        extendoSubsystem.telemetry(telemetry)
        teleSpace()
        telemetry.addData("Timer", "%.1f", timer.seconds()) //shows current time
        telemetry.addData("Loops", "%.1f", loops)
        telemetry.addData("Current LPS", "%.1f", LPS)
        telemetry.addData("Refresh Rate", "%.1f", rrPS)
        teleSpace()
        telemetry.addData("Color", lights.currentColor())
        telemetry.addData("Status", statusVal) //shows current status
        teleSpace()
        telemetry.addData("Version", currentVersion)
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

