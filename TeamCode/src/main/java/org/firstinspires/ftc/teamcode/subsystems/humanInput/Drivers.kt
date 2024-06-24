package org.firstinspires.ftc.teamcode.subsystems.humanInput

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.avoidance.AvoidanceSubsystem
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.ClawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.EndgameSubsystem
import org.firstinspires.ftc.teamcode.utilClass.Driver
import org.firstinspires.ftc.teamcode.utilClass.DriverAid
import org.firstinspires.ftc.teamcode.utilClass.Operator
import org.firstinspires.ftc.teamcode.utilClass.objects.DriveType


object Drivers {

    private var drivers = mutableListOf(
        Driver(AllDrivers.Camden, AvoidanceSubsystem.AvoidanceTypes.STOP, DriveType.ROBOT_CENTRIC),
        Driver(AllDrivers.Grady, AvoidanceSubsystem.AvoidanceTypes.PUSH, DriveType.FIELD_CENTRIC),
        Driver(AllDrivers.Michael, AvoidanceSubsystem.AvoidanceTypes.OFF, DriveType.ROBOT_CENTRIC),
        Driver(AllDrivers.Graden, AvoidanceSubsystem.AvoidanceTypes.PUSH, DriveType.ROBOT_CENTRIC),
        Driver(AllDrivers.Delaney, AvoidanceSubsystem.AvoidanceTypes.OFF, DriveType.ROBOT_CENTRIC),
        Driver(AllDrivers.Child, AvoidanceSubsystem.AvoidanceTypes.OFF, DriveType.ROBOT_CENTRIC),
    )
    val others = mutableListOf(
        Operator(AllDrivers.Camden),
        Operator(AllDrivers.Grady),
        Operator(AllDrivers.Michael),
        Operator(AllDrivers.Graden),
        Operator(AllDrivers.Delaney),
        Operator(AllDrivers.Child)
    )
    private val baseDriver = drivers.indexOfFirst { it.name == AllDrivers.Grady }
    private val baseOther =
        others.indexOfFirst { it.name == AllDrivers.Camden } //list integer of base driver and other controls
    private var dIndex = baseDriver
    private var oIndex = baseOther
    var currDriver = drivers[baseDriver]
    var currOther = others[baseOther] //list string of driver and other controls
    private var currentAvoidance = currDriver.defaultAvoidance
    var currentFieldCentric = currDriver.fieldCentric
    private var optionsHigh1 = false
    private var shareHigh1 = false
    private var optionsHigh2 = false
    private var shareHigh2 = false
    private var slowModeButtonDown = false
    private var planeButtonDown = false

    enum class AllDrivers {
        Camden,
        Grady,
        Michael,
        Graden,
        Delaney,
        Child
    }

    fun bindDriverButtons(
        myOpMode: OpMode,
        driveSubsystem: DriveSubsystem,
        clawSubsystem: ClawSubsystem,
        endgameSubsystem: EndgameSubsystem
    ): AvoidanceSubsystem.AvoidanceTypes {
        val drive = driveSubsystem.drive
        // "Camden", "Grady", "Michael","Graden", "Delaney", "Child"
        if (currDriver.name == AllDrivers.Grady) { //grady
            //slowmode
            if (myOpMode.gamepad1.circle && !slowModeButtonDown && !driveSubsystem.slowModeIsOn) {
                driveSubsystem.slowModeIsOn = true
            } else if (myOpMode.gamepad1.circle && !slowModeButtonDown && driveSubsystem.slowModeIsOn) {
                driveSubsystem.slowModeIsOn = false
            }
            slowModeButtonDown = myOpMode.gamepad1.circle
            if (myOpMode.gamepad1.triangle && !planeButtonDown && !endgameSubsystem.planeReleased) {
                endgameSubsystem.shoot()
                endgameSubsystem.planeReleased = true
            } else if (myOpMode.gamepad1.triangle && !planeButtonDown && endgameSubsystem.planeReleased) {
                endgameSubsystem.resetAirplane()
                endgameSubsystem.planeReleased = false
            }
            planeButtonDown = myOpMode.gamepad1.triangle
            if (myOpMode.gamepad1.right_trigger > 0) {
                endgameSubsystem.retract()
            } else if (myOpMode.gamepad1.left_trigger > 0) {
                endgameSubsystem.extend()
            } else {
                endgameSubsystem.stopLift()
            }
            DriverAid.doDriverAid(
                driveSubsystem,
                myOpMode.gamepad1.right_bumper,
                myOpMode.gamepad1.dpad_up,
                myOpMode.gamepad1.dpad_right,
                myOpMode.gamepad1.cross,
            )
        }

        if (currDriver.name === AllDrivers.Camden) { //Camden
            if (myOpMode.gamepad1.right_bumper) {
                val packet = TelemetryPacket()
                driveSubsystem.cancelableFollowing.run(packet)
//                runBlocking(
//                    drive.actionBuilder(drive.pose)
//                        .splineTo(Vector2d(0.0, 0.0), 0.0)
//                        .build()
//                )
            }
        }
        if (currDriver.name == AllDrivers.Michael) { //Michael
        }
        if (currDriver.name == AllDrivers.Graden) { //Graden
            //slowmode
            if (myOpMode.gamepad1.circle && !slowModeButtonDown && !driveSubsystem.slowModeIsOn) {
                driveSubsystem.slowModeIsOn = true
            } else if (myOpMode.gamepad1.circle && !slowModeButtonDown && driveSubsystem.slowModeIsOn) {
                driveSubsystem.slowModeIsOn = false
            }
            slowModeButtonDown = myOpMode.gamepad1.circle
            if (myOpMode.gamepad1.triangle && !planeButtonDown && !endgameSubsystem.planeReleased) {
                endgameSubsystem.shoot()
                endgameSubsystem.planeReleased = true
            } else if (myOpMode.gamepad1.triangle && !planeButtonDown && endgameSubsystem.planeReleased) {
                endgameSubsystem.resetAirplane()
                endgameSubsystem.planeReleased = false
            }
            planeButtonDown = myOpMode.gamepad1.triangle
            if (myOpMode.gamepad1.right_trigger > 0) {
                endgameSubsystem.retract()
            } else if (myOpMode.gamepad1.left_trigger > 0) {
                endgameSubsystem.extend()
            } else {
                endgameSubsystem.stopLift()
            }
            DriverAid.doDriverAid(
                driveSubsystem,
                myOpMode.gamepad1.right_bumper,
                myOpMode.gamepad1.dpad_up,
                myOpMode.gamepad1.dpad_right,
                myOpMode.gamepad1.cross
            )
        }
        if (currDriver.name == AllDrivers.Delaney) { //Delaney
            //slowmode
            if (myOpMode.gamepad1.circle && !slowModeButtonDown && !driveSubsystem.slowModeIsOn) {
                driveSubsystem.slowModeIsOn = true
            } else if (myOpMode.gamepad1.circle && !slowModeButtonDown && driveSubsystem.slowModeIsOn) {
                driveSubsystem.slowModeIsOn = false
            }
            slowModeButtonDown = myOpMode.gamepad1.circle
        }
        if (currDriver.name == AllDrivers.Child) { //Child
            driveSubsystem.slowModeIsOn = true
        }
        return currentAvoidance
    }

    fun switchProfile(myOpMode: OpMode) {
        //driver
        var driverChanged = false
        var otherChanged = false
        if (myOpMode.gamepad1.options && !optionsHigh1 && (!myOpMode.gamepad1.circle || !myOpMode.gamepad1.cross) && (!myOpMode.gamepad2.circle || !myOpMode.gamepad2.cross)) {
            if (dIndex == drivers.size - 1) {
                dIndex = 0
            } else {
                dIndex++
            }
            currDriver = drivers[dIndex]
            driverChanged = true
        }
        optionsHigh1 = myOpMode.gamepad1.options
        if (myOpMode.gamepad1.share && !shareHigh1 && (!myOpMode.gamepad1.circle || !myOpMode.gamepad1.cross) && (!myOpMode.gamepad2.circle || !myOpMode.gamepad2.cross)) {
            if (dIndex == 0) {
                dIndex = drivers.size - 1
            } else {
                dIndex--
            }
            currDriver = drivers[dIndex]
            driverChanged = true
        }
        shareHigh1 = myOpMode.gamepad1.share
        //other
        if (myOpMode.gamepad2.options && !optionsHigh2 && (!myOpMode.gamepad2.circle || !myOpMode.gamepad2.cross) && (!myOpMode.gamepad1.circle || !myOpMode.gamepad1.cross)) {
            if (oIndex == others.size - 1) {
                oIndex = 0
            } else {
                oIndex++
            }
            currOther = others[oIndex]
            otherChanged = true
        }
        optionsHigh2 = myOpMode.gamepad2.options
        if (myOpMode.gamepad2.share && !shareHigh2 && (!myOpMode.gamepad2.circle || !myOpMode.gamepad2.cross) && (!myOpMode.gamepad1.circle || !myOpMode.gamepad1.cross)) {
            if (oIndex == 0) {
                oIndex = others.size - 1
            } else {
                oIndex--
            }
            currOther = others[oIndex]
            otherChanged = true
        }
        shareHigh2 = myOpMode.gamepad2.share
        if (driverChanged) {
            currentAvoidance = currDriver.defaultAvoidance
            currentFieldCentric = currDriver.fieldCentric
        }
    }
}