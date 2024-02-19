//import
package org.firstinspires.ftc.teamcode.opModes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
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
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Drivers
import org.firstinspires.ftc.teamcode.Drivers.bindDriverButtons
import org.firstinspires.ftc.teamcode.Enums.StartDist
import org.firstinspires.ftc.teamcode.Operator.bindOtherButtons
import org.firstinspires.ftc.teamcode.UtilClass.FileWriterFTC
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.IsBusy
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.LoopTime
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.varConfig
import org.firstinspires.ftc.teamcode.extensions.BlinkExtensions.currentColor
import org.firstinspires.ftc.teamcode.extensions.BlinkExtensions.init
import org.firstinspires.ftc.teamcode.extensions.Extensions.ledIND
import org.firstinspires.ftc.teamcode.extensions.Extensions.potentAngle
import org.firstinspires.ftc.teamcode.extensions.MotorExtensions.init
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.currentVoltage
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.init
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.lowVoltage
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.calcFlipPose
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.init
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.servoFlipVal
import org.firstinspires.ftc.teamcode.opModes.DistanceStorage.Companion.totalDist
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage
import java.io.FileWriter

open class HardwareConfig(opmode: LinearOpMode) {
    private var slowMult: Int = varConfig.slowMult
    private var slowPower = 0
    private var xControl = 0.0
    private var yControl = 0.0
    private var frontRightPower = 0.0
    private var frontLeftPower = 0.0
    private var backRightPower = 0.0
    private var backLeftPower = 0.0
    private var gamepadX = 0.0
    private var gamepadY = 0.0
    private var gamepadHypot = 0.0
    private var controllerAngle = 0.0
    private var robotDegree = 0.0
    private var movementDegree = 0.0
    var reverse = false

    init {
        myOpMode = opmode
    }

    //code to run all drive functions
    fun doBulk() {
        once(myOpMode) //runs once
        periodically() //runs every loop
        loopTimeCalculations()
        bindDriverButtons(myOpMode, drive)
        bindOtherButtons(myOpMode, drive)
        if (multipleDrivers) {
            Drivers.switchProfile(myOpMode)
        }
        drive(Drivers.fieldCentric)
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

    fun periodically() {
        if (LoopTime.useLoopTime) {
            if (loops % LoopTime.loopInterval == 0.0) { // happens every loopInterval, loops
                refreshRate++
            }
        }
    }

    fun drive(fieldCentric: Boolean) {
        if (fieldCentric) {
            gamepadX =
                myOpMode.gamepad1.left_stick_x.toDouble() //get the x val of left stick and store
            gamepadY =
                -myOpMode.gamepad1.left_stick_y.toDouble() //get the y val of left stick and store
            gamepadHypot = Range.clip(Math.hypot(gamepadX, gamepadY), 0.0, 1.0) //get the
            // hypotenuse of the x and y values,clip it to a max of 1 and store
            controllerAngle = Math.toDegrees(
                Math.atan2(
                    gamepadY,
                    gamepadX
                )
            ) //Get the angle of the controller stick using arc tangent
            robotDegree = Math.toDegrees(drive.poseEstimate.heading) // change to imu
            movementDegree =
                controllerAngle - robotDegree //get the movement degree based on the controller vs robot angle
            xControl =
                Math.cos(Math.toRadians(movementDegree)) * gamepadHypot //get the x value of the movement
            yControl =
                Math.sin(Math.toRadians(movementDegree)) * gamepadHypot //get the y value of the movement
            val turn: Double = -myOpMode.gamepad1.right_stick_x.toDouble()
            frontRightPower =
                (yControl * Math.abs(yControl) - xControl * Math.abs(xControl) + turn) / slowPower
            backRightPower =
                (yControl * Math.abs(yControl) + xControl * Math.abs(xControl) + turn) / slowPower
            frontLeftPower =
                (yControl * Math.abs(yControl) + xControl * Math.abs(xControl) - turn) / slowPower
            backLeftPower =
                (yControl * Math.abs(yControl) - xControl * Math.abs(xControl) - turn) / slowPower
        } else {
//            reverse = myOpMode.gamepad1.touchpad_finger_1_x > 0.5;//0,1 left to right
//            reversed = reverse;
            yControl = -myOpMode.gamepad1.left_stick_y.toDouble()
            xControl = myOpMode.gamepad1.left_stick_x.toDouble()
            if (reverse) {
                yControl = -yControl
                xControl = -xControl
            }
            val turn: Double = -myOpMode.gamepad1.right_stick_x.toDouble()
            slowPower = if (slowModeIsOn) {
                slowMult
            } else {
                1
            }
            frontRightPower = (yControl - xControl + turn) / slowPower
            backRightPower = (yControl + xControl + turn) / slowPower
            frontLeftPower = (yControl + xControl - turn) / slowPower
            backLeftPower = (yControl - xControl - turn) / slowPower
        }
        drive.update()
        updateDistTraveled(PoseStorage.currentPose, drive.poseEstimate)
        FileWriterFTC.writeToFile(
            fileWriter,
            drive.poseEstimate.x.toInt(),
            drive.poseEstimate.y.toInt()
        )
        PoseStorage.currentPose = drive.poseEstimate
    }

    fun updateDistTraveled(before: Pose2d, after: Pose2d) {
        val x = after.x - before.x
        val y = after.y - before.y
        val dist = Math.sqrt(x * x + y * y)
        thisDist += dist
        totalDist += dist
    }

    fun power() { // put all set power here
        if (!IsBusy.isAutoInTeleop) {
            motorFrontLeft.power = frontLeftPower
            motorBackLeft.power = backLeftPower
            motorFrontRight.power = frontRightPower
            motorBackRight.power = backRightPower
        }
        motorLift.power = liftPower
        motorExtension.power = extensionPower
        motorRotation.power = rotationPower
        flipServo.calcFlipPose(servoFlipVal.toDouble())
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
        //        telemetry.addData("power", extensionPower);
//        telemetry.addData("power R", rotationPower);
        if (reversed) {
            telemetry.addData("reversed", "")
        }
        if (slowModeIsOn) {
            telemetry.addData("slowMode", "")
        }
        telemetry.addData("Extension", motorExtension.currentPosition)
        telemetry.addData("Rotation", motorRotation.currentPosition)
        teleSpace()
        telemetry.addData("thisDistance (in)", "%.1f", thisDist)
        telemetry.addData("totalDistance (in)", "%.1f", totalDist)
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

    companion object {
        var useFileWriter: Boolean = varConfig.useFileWriter
        var multipleDrivers: Boolean = varConfig.multipleDrivers
        var statusVal = "OFFLINE"
        lateinit var claw1: Servo
        lateinit var claw2: Servo
        lateinit var flipServo: Servo
        lateinit var airplaneServo: Servo
        lateinit var motorFrontLeft: DcMotor
        lateinit var motorBackLeft: DcMotor
        lateinit var motorFrontRight: DcMotor
        lateinit var motorBackRight: DcMotor
        lateinit var motorLift: DcMotor
        lateinit var motorExtension: DcMotor
        lateinit var motorRotation: DcMotor
        lateinit var lights: RevBlinkinLedDriver
        var slowModeIsOn = false
        var reversed = false
        var liftPower = 0.0
        var extensionPower = 0.0
        var rotationPower = 0.0
        var loops = 0.0
        var LPS = 0.0
        var refreshRate = 0.0
        var rrPS = 0.0
        var pastRefreshRate = refreshRate
        var pastSecondLoops = 0.0
        var pastTimeRR = 0.0
        var lastTimeOpen = 0.0
        var pastUseLoopTime: Boolean = LoopTime.useLoopTime
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
        lateinit var fileWriter: FileWriter
        private lateinit var myOpMode: LinearOpMode
        var once = false
        var extensionPIDF = PIDFController(0.0, 0.0, 0.0, 0.0)
        var rotationPIDF = PIDFController(0.0, 0.0, 0.0, 0.0)
        lateinit var startDist: StartDist
        const val currentVersion = "5.5.0"

        //init
        fun init(ahwMap: HardwareMap, auto: Boolean) {
            val telemetry: Telemetry =
                MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().telemetry)
            thisDist = 0.0
            once = false
            FileWriterFTC.setUpFile(fileWriter)
            updateStatus("Initializing")
            drive = MecanumDrive(ahwMap)
            drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            drive.poseEstimate = PoseStorage.currentPose
            val timer = ElapsedTime() //declaring the runtime variable
            vSensor.init(ahwMap, "Expansion Hub 2")
            lights.init(ahwMap, "blinkin")
            // rev potentiometer //analog
            potentiometer.init(ahwMap, "potent")
            green1.init(ahwMap, "green1")
            green2.init(ahwMap, "green2")
            green3.init(ahwMap, "green3")
            green4.init(ahwMap, "green4")
            red1.init(ahwMap, "red1")
            red2.init(ahwMap, "red2")
            red3.init(ahwMap, "red3")
            red4.init(ahwMap, "red4")
            // Declare our motors
            motorFrontLeft.init(ahwMap, "motorFrontLeft", DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            motorBackLeft.init(
                ahwMap,
                "motorBackLeft",
                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                DcMotorSimple.Direction.REVERSE
            )
            motorFrontRight.init(ahwMap, "motorFrontRight", DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            motorBackRight.init(ahwMap, "motorBackRight", DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            motorLift.init(
                ahwMap,
                "lift",
                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                DcMotorSimple.Direction.REVERSE
            )
            motorExtension.init(ahwMap, "slideMotor", DcMotor.RunMode.RUN_USING_ENCODER)
            motorRotation.init(
                ahwMap,
                "flipperMotor",
                DcMotor.RunMode.RUN_USING_ENCODER,
                DcMotorSimple.Direction.REVERSE
            )
            claw1.init(ahwMap, "claw1")
            claw2.init(ahwMap, "claw2")
            flipServo.init(ahwMap, "flipServo")
            airplaneServo.init(ahwMap, "airplaneServo")
            ServoUtil.closeClaw(claw1)
            ServoUtil.closeClaw(claw2)
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
                telemetry.addData("Random", autoHardware.autonomousRandom)
            }
            if (vSensor.lowVoltage()) {
                telemetry.addData("lowBattery", "true")
            }
            if (!auto) {
                telemetry.update()
            }
        }

        fun loopTimeCalculations() {
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
            //        if (LPS != lastLPS) {
//            LPSList.add(LPS);
//            LPSAverage = LPSList.stream().mapToDouble(val -> val).average().orElse(0.0);
//            lastLPS = LPS;
//        }
            if (refreshRate != pastRefreshRate) {
                rrPS = timer.seconds() - pastTimeRR
                pastRefreshRate = refreshRate
                pastTimeRR = timer.seconds()
            }
        }

        fun updateStatus(status: String) {
            statusVal = status
        }
    }
}

