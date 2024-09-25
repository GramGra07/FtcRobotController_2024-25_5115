package org.firstinspires.ftc.teamcode.subsystems.humanInput

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.extensions.GamepadExtensions
import org.firstinspires.ftc.teamcode.extensions.GamepadExtensions.buttonJustPressed
import org.firstinspires.ftc.teamcode.ggutil.AvoidanceSubsystem
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.LiftSubsystem
import org.firstinspires.ftc.teamcode.utilClass.DriverAid
import org.firstinspires.ftc.teamcode.utilClass.objects.DriveType
import org.firstinspires.ftc.teamcode.utilClass.objects.Driver
import org.firstinspires.ftc.teamcode.utilClass.objects.Operator

object Drivers {

    private var drivers = mutableListOf(
        Driver(AllDrivers.Camden, AvoidanceSubsystem.AvoidanceTypes.OFF, DriveType.ROBOT_CENTRIC),
        Driver(AllDrivers.Grady, AvoidanceSubsystem.AvoidanceTypes.OFF, DriveType.ROBOT_CENTRIC),
        Driver(AllDrivers.Michael, AvoidanceSubsystem.AvoidanceTypes.OFF, DriveType.ROBOT_CENTRIC),
        Driver(AllDrivers.Graden, AvoidanceSubsystem.AvoidanceTypes.OFF, DriveType.ROBOT_CENTRIC),
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
    private var dIndex = baseDriver - 1
    private var oIndex = baseOther
    var currDriver = drivers[baseDriver]
    var currOther = others[baseOther] //list string of driver and other controls
    var currentFieldCentric = currDriver.fieldCentric

//    private var optionsHigh1 = false
//    private var shareHigh1 = false
//    private var optionsHigh2 = false
//    private var shareHigh2 = false
//    private var slowModeButtonDown = false
//    private var planeButtonDown = false

    enum class AllDrivers {
        Camden,
        Grady,
        Michael,
        Graden,
        Delaney,
        Child
    }

    var slowModeButtonDown = false
    var planeButtonDown = false
    fun bindDriverButtons(
        myOpMode: OpMode,
        driveSubsystem: DriveSubsystem,
//        liftSubsystem: LiftSubsystem,
    ) {
        // "Camden", "Grady", "Michael","Graden", "Delaney", "Child"
        if (currDriver.name == AllDrivers.Grady) { //grady
            //slowmode
            if (myOpMode.gamepad1.buttonJustPressed(GamepadExtensions.PushButtons.CIRCLE, 1)) {
                driveSubsystem.slowModeIsOn = !driveSubsystem.slowModeIsOn
            }
//
//            if (myOpMode.gamepad1.buttonJustPressed(GamepadExtensions.PushButtons.TRIANGLE, 1)) {
//                liftSubsystem.setPower(liftSubsystem.maxLiftExtension)
//            } else if (myOpMode.gamepad1.buttonJustPressed(GamepadExtensions.PushButtons.CROSS, 1)) {
//                liftSubsystem.setPower(liftSubsystem.minLiftExtension)
//            } else {
//                liftSubsystem.stop()
//            }
            DriverAid.doDriverAid()
        }

        if (currDriver.name === AllDrivers.Camden) { //Camden
        }
        if (currDriver.name == AllDrivers.Michael) { //Michael
        }
        if (currDriver.name == AllDrivers.Graden) { //Graden
        }
        if (currDriver.name == AllDrivers.Delaney) { //Delaney

        }
        if (currDriver.name == AllDrivers.Child) { //Child
//            driveSubsystem.slowModeIsOn = true
        }
    }

    var optionsHigh1 = false
    var optionsHigh2 = false
    var shareHigh1 = false
    var shareHigh2 = false

    fun switchProfile(
        myOpMode: OpMode,
    ) {
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
            currentFieldCentric = currDriver.fieldCentric
        }
        if (otherChanged) {

        }
    }
}