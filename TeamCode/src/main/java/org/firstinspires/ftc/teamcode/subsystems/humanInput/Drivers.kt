package org.firstinspires.ftc.teamcode.subsystems.humanInput

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.extensions.GamepadExtensions
import org.firstinspires.ftc.teamcode.extensions.GamepadExtensions.buttonJustPressed
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.utilClass.objects.DriveType

object Drivers {
    var currentFieldCentric = DriveType.ROBOT_CENTRIC

//    private var optionsHigh1 = false
//    private var shareHigh1 = false
//    private var optionsHigh2 = false
//    private var shareHigh2 = false
//    private var slowModeButtonDown = false
//    private var planeButtonDown = false

    fun bindDriverButtons(
        myOpMode: OpMode,
        driveSubsystem: DriveSubsystem,
//        liftSubsystem: LiftSubsystem,
    ) {
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
    }
}