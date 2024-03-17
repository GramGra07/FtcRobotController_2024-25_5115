package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.util.Angle.normDelta
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.UtilClass.DriverAid
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.EndgameSubsystem

object Drivers {
    private val driverControls =
        arrayOf("Camden", "Grady", "Michael", "Graden", "Delaney", "Child")
    val otherControls = driverControls
    const val baseDriver = 1
    const val baseOther = 0 //list integer of base driver and other controls
    var dIndex: Int = baseDriver
    var oIndex: Int = baseOther //list integer of driver and other controls
    var currDriver = driverControls[dIndex]
    var currOther = otherControls[oIndex] //list string of driver and other controls
    var fieldCentric = false
    private var liftConnected = false
    private var optionsHigh1 = false
    private var shareHigh1 = false
    private var optionsHigh2 = false
    private var shareHigh2 = false
    private var slowModeButtonDown = false
    private var planeButtonDown = false
    private var planeReleased = true
    fun bindDriverButtons(
        myOpMode: OpMode,
        driveSubsystem: DriveSubsystem,
        clawSubsystem: ClawSubsystem,
        endgameSubsystem: EndgameSubsystem
    ) {
        val drive = driveSubsystem.drive!!
        // "Camden", "Grady", "Michael","Graden", "Delaney", "Child"
        if (currDriver === driverControls[1]) { //grady
            fieldCentric = true
            //slowmode
//            gamepad1.wasJustReleased(GamepadKeys.Button.B);
            if (myOpMode.gamepad1.circle && !slowModeButtonDown && !driveSubsystem.slowModeIsOn) {
                driveSubsystem.slowModeIsOn = true
            } else if (myOpMode.gamepad1.circle && !slowModeButtonDown && driveSubsystem.slowModeIsOn) {
                driveSubsystem.slowModeIsOn = false
            }
            slowModeButtonDown = myOpMode.gamepad1.circle
            //            gamepad1.wasJustReleased(GamepadKeys.Button.Y);
            if (myOpMode.gamepad1.triangle && !planeButtonDown && !endgameSubsystem.planeReleased) {
                endgameSubsystem.shoot()
                endgameSubsystem.planeReleased = true
            } else if (myOpMode.gamepad1.triangle && !planeButtonDown && endgameSubsystem.planeReleased) {
                endgameSubsystem.resetAirplane()
                endgameSubsystem.planeReleased = false
            }
            planeButtonDown = myOpMode.gamepad1.triangle
            //            gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            // gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
            if (myOpMode.gamepad1.right_trigger > 0) {
//                HardwareConfig.liftPower = -Limits.liftMax
                endgameSubsystem.retract()
            } else if (myOpMode.gamepad1.left_trigger > 0) {
                endgameSubsystem.extend()
//                HardwareConfig.liftPower = Limits.liftMax
            } else {
                endgameSubsystem.stopLift()
//                HardwareConfig.liftPower = 0.0
            }
            // using ftc lib
//            gamepad1.getButton(GamepadKeys.Button.RIGHT_BUMPER);
//            gamepad1.getButton(GamepadKeys.Button.DPAD_UP);
//            gamepad1.getButton(GamepadKeys.Button.DPAD_RIGHT);
//            gamepad1.getButton(GamepadKeys.Button.A);
            DriverAid.doDriverAid(
                driveSubsystem,
                myOpMode.gamepad1.right_bumper,
                myOpMode.gamepad1.dpad_up,
                myOpMode.gamepad1.dpad_right
            )
        }

        if (currDriver === driverControls[0]) { //Camden
            fieldCentric = false
        }
        if (currDriver === driverControls[2]) { //Michael
            fieldCentric = false
        }
        if (currDriver === driverControls[3]) { //Graden
            fieldCentric = false
            //slowmode
//            gamepad1.wasJustReleased(GamepadKeys.Button.B);
            if (myOpMode.gamepad1.circle && !slowModeButtonDown && !driveSubsystem.slowModeIsOn) {
                driveSubsystem.slowModeIsOn = true
            } else if (myOpMode.gamepad1.circle && !slowModeButtonDown && driveSubsystem.slowModeIsOn) {
                driveSubsystem.slowModeIsOn = false
            }
            slowModeButtonDown = myOpMode.gamepad1.circle
            //            gamepad1.wasJustReleased(GamepadKeys.Button.Y);
            if (myOpMode.gamepad1.triangle && !planeButtonDown && !endgameSubsystem.planeReleased) {
                endgameSubsystem.shoot()
                endgameSubsystem.planeReleased = true
            } else if (myOpMode.gamepad1.triangle && !planeButtonDown && endgameSubsystem.planeReleased) {
                endgameSubsystem.resetAirplane()
                endgameSubsystem.planeReleased = false
            }
            planeButtonDown = myOpMode.gamepad1.triangle
            //            gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            // gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
            if (myOpMode.gamepad1.right_trigger > 0) {
//                HardwareConfig.liftPower = -Limits.liftMax
                endgameSubsystem.retract()
            } else if (myOpMode.gamepad1.left_trigger > 0) {
                endgameSubsystem.extend()
//                HardwareConfig.liftPower = Limits.liftMax
            } else {
                endgameSubsystem.stopLift()
//                HardwareConfig.liftPower = 0.0
            }
            // using ftc lib
//            gamepad1.getButton(GamepadKeys.Button.RIGHT_BUMPER);
//            gamepad1.getButton(GamepadKeys.Button.DPAD_UP);
//            gamepad1.getButton(GamepadKeys.Button.DPAD_RIGHT);
//            gamepad1.getButton(GamepadKeys.Button.A);
            DriverAid.doDriverAid(
                driveSubsystem,
                myOpMode.gamepad1.right_bumper,
                myOpMode.gamepad1.dpad_up,
                myOpMode.gamepad1.dpad_right
            )
        }
        if (currDriver === driverControls[4]) { //Delaney
            fieldCentric = false
            //slowmode
            if (myOpMode.gamepad1.circle && !slowModeButtonDown && !driveSubsystem.slowModeIsOn) {
                driveSubsystem.slowModeIsOn = true
            } else if (myOpMode.gamepad1.circle && !slowModeButtonDown && driveSubsystem.slowModeIsOn) {
                driveSubsystem.slowModeIsOn = false
            }
            slowModeButtonDown = myOpMode.gamepad1.circle
            if (myOpMode.gamepad1.triangle) {
                driveSubsystem.slowModeIsOn = false
                driveSubsystem.isAutoInTeleop = true
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
                    .addDisplacementMarker {
                        clawSubsystem.openLeft()
                        clawSubsystem.update()
                    }
                    .addDisplacementMarker {
                        clawSubsystem.openRight()
                        clawSubsystem.update()
                    }
                    .splineToLinearHeading(
                        Pose2d(
                            drive.poseEstimate.x - 5, drive.poseEstimate.y - 10, normDelta(
                                Math.toRadians(45.0) - drive.poseEstimate.heading
                            )
                        ), normDelta(Math.toRadians(135.0) - drive.poseEstimate.heading)
                    )
                    .addDisplacementMarker {
                        clawSubsystem.closeBoth()
                        clawSubsystem.update()
                    }
                    .splineToLinearHeading(
                        PoseStorage.currentPose,
                        PoseStorage.currentPose.heading
                    )
                    .build())
            }
            if (myOpMode.gamepad1.cross) {
                driveSubsystem.isAutoInTeleop = true
                drive.breakFollowing()
            }
            if (!drive.isBusy) {
                driveSubsystem.isAutoInTeleop = false
            }
        }
        if (currDriver === driverControls[5]) { //Child
            fieldCentric = false
            driveSubsystem.slowModeIsOn = true
        }
    }

    fun switchProfile(myOpMode: OpMode) {
        //driver
        if (myOpMode.gamepad1.options && !optionsHigh1 && (!myOpMode.gamepad1.circle || !myOpMode.gamepad1.cross) && (!myOpMode.gamepad2.circle || !myOpMode.gamepad2.cross)) {
            if (dIndex == driverControls.size - 1) {
                dIndex = 0
            } else {
                dIndex++
            }
            currDriver = driverControls[dIndex]
        }
        optionsHigh1 = myOpMode.gamepad1.options
        if (myOpMode.gamepad1.share && !shareHigh1 && (!myOpMode.gamepad1.circle || !myOpMode.gamepad1.cross) && (!myOpMode.gamepad2.circle || !myOpMode.gamepad2.cross)) {
            if (dIndex == 0) {
                dIndex = driverControls.size - 1
            } else {
                dIndex--
            }
            currDriver = driverControls[dIndex]
        }
        shareHigh1 = myOpMode.gamepad1.share
        //other
        if (myOpMode.gamepad2.options && !optionsHigh2 && (!myOpMode.gamepad2.circle || !myOpMode.gamepad2.cross) && (!myOpMode.gamepad1.circle || !myOpMode.gamepad1.cross)) {
            if (oIndex == otherControls.size - 1) {
                oIndex = 0
            } else {
                oIndex++
            }
            currOther = otherControls[oIndex]
        }
        optionsHigh2 = myOpMode.gamepad2.options
        if (myOpMode.gamepad2.share && !shareHigh2 && (!myOpMode.gamepad2.circle || !myOpMode.gamepad2.cross) && (!myOpMode.gamepad1.circle || !myOpMode.gamepad1.cross)) {
            if (oIndex == 0) {
                oIndex = otherControls.size - 1
            } else {
                oIndex--
            }
            currOther = otherControls[oIndex]
        }
        shareHigh2 = myOpMode.gamepad2.share
        if (currDriver === driverControls[driverControls.size - 1]) {
            currOther = driverControls[driverControls.size - 1]
        }
    }
}