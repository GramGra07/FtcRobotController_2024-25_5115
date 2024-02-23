package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.util.Angle.normDelta
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.UtilClass.DriverAid
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.IsBusy
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.airplaneServo
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage

object Drivers {
    private val driverControls =
        arrayOf("Camden", "Grady", "Michael", "Graden", "Delaney", "Child")
    val otherControls = driverControls
    const val baseDriver = 1
    const val baseOther = 0 //list integer of base driver and other controls
    var currDriver = driverControls[DriverIndex.dIndex]
    var currOther = otherControls[DriverIndex.oIndex] //list string of driver and other controls
    var fieldCentric = false
    private var liftConnected = false
    private var optionsHigh1 = false
    private var shareHigh1 = false
    private var optionsHigh2 = false
    private var shareHigh2 = false
    private var slowModeButtonDown = false
    private var planeButtonDown = false
    private var planeReleased = true
    fun bindDriverButtons(myOpMode: OpMode, drive: MecanumDrive) {
        // "Camden", "Grady", "Michael","Graden", "Delaney", "Child"
        if (currDriver === driverControls[1]) { //grady
            fieldCentric = false
            //slowmode
//            gamepad1.wasJustReleased(GamepadKeys.Button.B);
            if (myOpMode.gamepad1.circle && !slowModeButtonDown && !HardwareConfig.slowModeIsOn) {
                HardwareConfig.slowModeIsOn = true
            } else if (myOpMode.gamepad1.circle && !slowModeButtonDown && HardwareConfig.slowModeIsOn) {
                HardwareConfig.slowModeIsOn = false
            }
            slowModeButtonDown = myOpMode.gamepad1.circle
            //            gamepad1.wasJustReleased(GamepadKeys.Button.Y);
            if (myOpMode.gamepad1.triangle && !planeButtonDown && !planeReleased) {
                ServoUtil.releaseAirplane(airplaneServo)
                planeReleased = true
            } else if (myOpMode.gamepad1.triangle && !planeButtonDown && planeReleased) {
                ServoUtil.resetAirplane(airplaneServo)
                planeReleased = false
            }
            planeButtonDown = myOpMode.gamepad1.triangle
            //            gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            // gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
            if (myOpMode.gamepad1.right_trigger > 0) {
                HardwareConfig.liftPower = -Limits.liftMax
            } else if (myOpMode.gamepad1.left_trigger > 0) {
                HardwareConfig.liftPower = Limits.liftMax
            } else {
                HardwareConfig.liftPower = 0.0
            }
            if (liftConnected) {
                HardwareConfig.extensionPower = HardwareConfig.liftPower
            }
            // using ftc lib
//            gamepad1.getButton(GamepadKeys.Button.RIGHT_BUMPER);
//            gamepad1.getButton(GamepadKeys.Button.DPAD_UP);
//            gamepad1.getButton(GamepadKeys.Button.DPAD_RIGHT);
//            gamepad1.getButton(GamepadKeys.Button.A);
            DriverAid.doDriverAid(
                drive,
                myOpMode.gamepad1.right_bumper,
                myOpMode.gamepad1.dpad_up,
                myOpMode.gamepad1.dpad_right,
                myOpMode.gamepad1.cross
            )
        }

        if (currDriver === driverControls[0]) { //Camden
        }
        if (currDriver === driverControls[2]) { //Michael
        }
        if (currDriver === driverControls[3]) { //Graden
            fieldCentric = true
            if (myOpMode.gamepad1.circle && !slowModeButtonDown && !HardwareConfig.slowModeIsOn) {
                HardwareConfig.slowModeIsOn = true
            } else if (myOpMode.gamepad1.circle && !slowModeButtonDown && HardwareConfig.slowModeIsOn) {
                HardwareConfig.slowModeIsOn = false
            }
            slowModeButtonDown = myOpMode.gamepad1.circle
        }
        if (currDriver === driverControls[4]) { //Delaney
            fieldCentric = false
            //slowmode
            if (myOpMode.gamepad1.circle && !slowModeButtonDown && !HardwareConfig.slowModeIsOn) {
                HardwareConfig.slowModeIsOn = true
            } else if (myOpMode.gamepad1.circle && !slowModeButtonDown && HardwareConfig.slowModeIsOn) {
                HardwareConfig.slowModeIsOn = false
            }
            slowModeButtonDown = myOpMode.gamepad1.circle
            if (myOpMode.gamepad1.triangle) {
                HardwareConfig.slowModeIsOn = false
                IsBusy.isAutoInTeleop = true
                drive.update()
                PoseStorage.currentPose = drive.poseEstimate
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.poseEstimate)
                    .splineToLinearHeading(
                        Pose2d(
                            drive.poseEstimate.x + 5, drive.poseEstimate.y + 10, normDelta(
                                Math.toRadians(135.0) - drive.poseEstimate.heading
                            )
                        ), normDelta(Math.toRadians(135.0) - drive.poseEstimate.heading)
                    )
                    .addDisplacementMarker { ServoUtil.openClaw(HardwareConfig.claw1) }
                    .addDisplacementMarker { ServoUtil.openClaw(HardwareConfig.claw2) }
                    .splineToLinearHeading(
                        Pose2d(
                            drive.poseEstimate.x - 5, drive.poseEstimate.y - 10, normDelta(
                                Math.toRadians(45.0) - drive.poseEstimate.heading
                            )
                        ), normDelta(Math.toRadians(135.0) - drive.poseEstimate.heading)
                    )
                    .addDisplacementMarker { ServoUtil.closeClaw(HardwareConfig.claw1) }
                    .addDisplacementMarker { ServoUtil.closeClaw(HardwareConfig.claw2) }
                    .splineToLinearHeading(
                        PoseStorage.currentPose,
                        PoseStorage.currentPose.heading
                    )
                    .build())
            }
            if (myOpMode.gamepad1.cross) {
                IsBusy.isAutoInTeleop = true
                drive.breakFollowing()
            }
            if (!drive.isBusy) {
                IsBusy.isAutoInTeleop = false
            }
        }
        if (currDriver === driverControls[5]) { //Child
            fieldCentric = false
            HardwareConfig.slowModeIsOn = true
        }
    }

    fun switchProfile(myOpMode: OpMode) {
        //driver
        if (myOpMode.gamepad1.options && !optionsHigh1) {
            if (DriverIndex.dIndex == driverControls.size - 1) {
                DriverIndex.dIndex = 0
            } else {
                DriverIndex.dIndex++
            }
            currDriver = driverControls[DriverIndex.dIndex]
        }
        optionsHigh1 = myOpMode.gamepad1.options
        if (myOpMode.gamepad1.share && !shareHigh1) {
            if (DriverIndex.dIndex == 0) {
                DriverIndex.dIndex = driverControls.size - 1
            } else {
                DriverIndex.dIndex--
            }
            currDriver = driverControls[DriverIndex.dIndex]
        }
        shareHigh1 = myOpMode.gamepad1.share
        //other
        if (myOpMode.gamepad2.options && !optionsHigh2) {
            if (DriverIndex.oIndex == otherControls.size - 1) {
                DriverIndex.oIndex = 0
            } else {
                DriverIndex.oIndex++
            }
            currOther = otherControls[DriverIndex.oIndex]
        }
        optionsHigh2 = myOpMode.gamepad2.options
        if (myOpMode.gamepad2.share && !shareHigh2) {
            if (DriverIndex.oIndex == 0) {
                DriverIndex.oIndex = otherControls.size - 1
            } else {
                DriverIndex.oIndex--
            }
            currOther = otherControls[DriverIndex.oIndex]
        }
        shareHigh2 = myOpMode.gamepad2.share
        if (currDriver === driverControls[7]) {
            currOther = driverControls[7]
        }
    }
}