package org.firstinspires.ftc.teamcode.opModes.autoSoftware

import org.firstinspires.ftc.teamcode.Enums.AutoRandom
import org.firstinspires.ftc.teamcode.Enums.EndPose
import org.firstinspires.ftc.teamcode.Enums.PathLong
import org.firstinspires.ftc.teamcode.Enums.StartDist
import org.firstinspires.ftc.teamcode.Limits
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.openClaw
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.AutoServoPositions
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.calcFlipPose
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig.Companion.flipServo
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.Companion.encoderDrive
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive
import org.gentrifiedApps.statemachineftc.StateMachine

object autoPatterns {
    fun place1Machine(drive: MecanumDrive): StateMachine<place1States> {
        val builder = StateMachine.Builder<place1States>()
        return builder
            .state(place1States.SPIKE_NAV)
            .onEnter(place1States.SPIKE_NAV) {
                flipServo.calcFlipPose(0.0)
//                generalPatterns.SpikeNav(drive, PathLong.NONE)
            }
            .whileState(place1States.SPIKE_NAV, { !drive.isBusy }) { drive.update() }
            .onExit(place1States.SPIKE_NAV) { openClaw(HardwareConfig.claw2) }
            .transition(place1States.SPIKE_NAV, { !drive.isBusy }, 0.0)
            .state(place1States.END_POSE)
            .onEnter(place1States.END_POSE) {
                flipServo.calcFlipPose(30.0)
                endPose.goToEndPose(EndPose.StartingPosition, drive)
            }
            .whileState(place1States.END_POSE, { !drive.isBusy }) { drive.update() }
            .transition(place1States.END_POSE, { !drive.isBusy }, 0.0)
            .stopRunning(place1States.STOP)
            .build()
    }

    var rotate = 0
    var extend = 0
    fun pixelParkMachine(
        drive: MecanumDrive,
        pathLong: PathLong,
        ePose: EndPose
    ): StateMachine<pixelParkStates> {
        val builder = StateMachine.Builder<pixelParkStates>()
        return builder
            .state(pixelParkStates.INIT)
            .onEnter(pixelParkStates.INIT) {}
            .transition(
                pixelParkStates.INIT,
                { true },
                0.0
            ) // if we want to make it delay before entering
            .state(pixelParkStates.SPIKE_NAV)
            .onEnter(pixelParkStates.SPIKE_NAV) {
                flipServo.calcFlipPose(0.0)
//                generalPatterns.SpikeNav(drive, pathLong)
            }
            .whileState(pixelParkStates.SPIKE_NAV, { !drive.isBusy }) { drive.update() }
            .onExit(pixelParkStates.SPIKE_NAV) {
                openClaw(HardwareConfig.claw2)
                flipServo.calcFlipPose(30.0)
                if (HardwareConfig.startDist == StartDist.LONG_SIDE) {
//                    BackdropTrajectories.blueMidOff = 7
                }
            }
            .transition(pixelParkStates.SPIKE_NAV, { !drive.isBusy }, 0.0)
            .state(pixelParkStates.BACKDROP)
            .onEnter(pixelParkStates.BACKDROP) {
                flipServo.calcFlipPose(30.0)
//                generalPatterns.navToBackdrop_Place(drive, pathLong, false)
            }
            .whileState(pixelParkStates.BACKDROP, { !drive.isBusy }) { drive.update() }
            .onExit(pixelParkStates.BACKDROP) {
                val clawOffset = 10
                if (HardwareConfig.startDist == StartDist.LONG_SIDE) {
                    rotate = Limits.autoRotation - 400
                    extend = Limits.autoExtension
                    //                        calculateFlipPose(AutoServoPositions.flipDown - clawOffset, flipServo);
                    encoderDrive(HardwareConfig.motorRotation, rotate, 1.0, drive)
                } else {
                    extend = Limits.autoExtension / 2
                }
                flipServo.calcFlipPose((AutoServoPositions.flipDown - clawOffset).toDouble())
                encoderDrive(HardwareConfig.motorExtension, extend, 1.0, drive)
                openClaw(HardwareConfig.claw1)
            }
            .transition(pixelParkStates.BACKDROP, { !drive.isBusy }, 0.0)
            .state(pixelParkStates.END_POSE)
            .onEnter(pixelParkStates.END_POSE) {
                encoderDrive(HardwareConfig.motorExtension, -extend, 0.5, drive)
                flipServo.calcFlipPose(60.0)
                if (ePose != EndPose.NONE) {
                    endPose.goToEndPose(ePose, drive)
                }
            }
            .whileState(pixelParkStates.END_POSE, { !drive.isBusy }) { drive.update() }
            .transition(pixelParkStates.END_POSE, { !drive.isBusy }, 0.0)
            .state(pixelParkStates.RETRACT)
            .onEnter(pixelParkStates.RETRACT) {
                if (HardwareConfig.startDist == StartDist.LONG_SIDE) {
                    encoderDrive(HardwareConfig.motorRotation, -rotate, 1.0, drive)
                }
            }
            .onExit(pixelParkStates.RETRACT) {}
            .transition(pixelParkStates.RETRACT, { !drive.isBusy }, 0.0)
            .stopRunning(pixelParkStates.STOP)
            .build()
    }

    fun cycleMachine(
        drive: MecanumDrive,
        pathLong: PathLong,
        ePose: EndPose
    ): StateMachine<cycleStates> {
        val builder = StateMachine.Builder<cycleStates>()
        return builder
            .state(cycleStates.INIT)
            .onEnter(cycleStates.INIT) {}
            .transition(
                cycleStates.INIT,
                { true },
                0.0
            ) // if we want to make it delay before entering
            .state(cycleStates.SPIKE_NAV)
            .onEnter(cycleStates.SPIKE_NAV) {
                flipServo.calcFlipPose(0.0)
//                generalPatterns.SpikeNav(drive, pathLong)
            }
            .whileState(cycleStates.SPIKE_NAV, { !drive.isBusy }) { drive.update() }
            .onExit(cycleStates.SPIKE_NAV) {
                openClaw(HardwareConfig.claw2)
                flipServo.calcFlipPose(30.0)
                if (HardwareConfig.startDist == StartDist.LONG_SIDE) {
//                    BackdropTrajectories.blueMidOff = 7
                }
            }
            .transition(cycleStates.SPIKE_NAV, { !drive.isBusy }, 0.0)
            .state(cycleStates.BACKDROP)
            .onEnter(cycleStates.BACKDROP) {
                flipServo.calcFlipPose(30.0)
//                generalPatterns.navToBackdrop_Place(drive, pathLong, false)
            }
            .whileState(cycleStates.BACKDROP, { !drive.isBusy }) { drive.update() }
            .onExit(cycleStates.BACKDROP) {
                val clawOffset = 10
                if (HardwareConfig.startDist == StartDist.LONG_SIDE) {
                    rotate = Limits.autoRotation - 400
                    extend = Limits.autoExtension
                    //                        calculateFlipPose(AutoServoPositions.flipDown - clawOffset, flipServo);
                    encoderDrive(HardwareConfig.motorRotation, rotate, 1.0, drive)
                } else {
                    extend = Limits.autoExtension / 2
                }
                flipServo.calcFlipPose((AutoServoPositions.flipDown - clawOffset).toDouble())
                encoderDrive(HardwareConfig.motorExtension, extend, 1.0, drive)
                openClaw(HardwareConfig.claw1)
            }
            .transition(cycleStates.BACKDROP, { !drive.isBusy }, 0.0)
            .state(cycleStates.RETRACT)
            .onEnter(cycleStates.RETRACT) {
                encoderDrive(HardwareConfig.motorExtension, -extend, 0.5, drive)
                flipServo.calcFlipPose(60.0)
                if (HardwareConfig.startDist == StartDist.LONG_SIDE) {
                    encoderDrive(HardwareConfig.motorRotation, -rotate, 1.0, drive)
                }
            }
            .onExit(cycleStates.RETRACT) {}
            .transition(cycleStates.RETRACT, { !drive.isBusy }, 0.0)
            .state(cycleStates.PICK1)
            .onEnter(cycleStates.PICK1) {
                autoHardware.autoRandomReliable = AutoRandom.MID
                cyclePatterns.pickFromSpot(drive, pathLong)
            }
            .whileState(cycleStates.PICK1, { !drive.isBusy }) { drive.update() }
            .onExit(cycleStates.PICK1) {}
            .transition(cycleStates.PICK1, { !drive.isBusy }, 0.0)
            .state(cycleStates.PLACE1)
            .onEnter(cycleStates.PLACE1) {
                AutoServoPositions.flipUp = 30
//                BackdropTrajectories.forwardOffset = 0
//                generalPatterns.navToBackdrop_Place(drive, pathLong, true)
            }
            .whileState(cycleStates.PLACE1, { !drive.isBusy }) { drive.update() }
            .onExit(cycleStates.PLACE1) {
//                BackdropTrajectories.forwardOffset = 0
                rotate = Limits.autoRotation - 200
                extend = Limits.autoExtension + 400
                encoderDrive(HardwareConfig.motorRotation, rotate, 1.0, drive)
                flipServo.calcFlipPose(0.0)
                encoderDrive(HardwareConfig.motorExtension, extend, 1.0, drive)
                openClaw(HardwareConfig.claw1)
                openClaw(HardwareConfig.claw2)
            }
            .transition(cycleStates.PLACE1, { !drive.isBusy }, 0.0)
            .state(cycleStates.END_POSE)
            .onEnter(cycleStates.END_POSE) {
                encoderDrive(HardwareConfig.motorExtension, -extend, 0.5, drive)
                flipServo.calcFlipPose(60.0)
                if (ePose != EndPose.NONE) {
                    endPose.goToEndPose(ePose, drive)
                }
            }
            .whileState(cycleStates.END_POSE, { !drive.isBusy }) { drive.update() }
            .transition(cycleStates.END_POSE, { !drive.isBusy }, 0.0)
            .state(cycleStates.RETRACT2)
            .onEnter(cycleStates.RETRACT2) {
                encoderDrive(
                    HardwareConfig.motorRotation,
                    -rotate,
                    1.0,
                    drive
                )
            }
            .onExit(cycleStates.RETRACT2) {}
            .transition(cycleStates.RETRACT2, { !drive.isBusy }, 0.0)
            .stopRunning(cycleStates.STOP)
            .build()
    }

    // contains different auto patterns for different tasks
    enum class place1States {
        INIT,
        SPIKE_NAV,
        END_POSE,
        STOP
    }

    // does two pixel and then goes to the end pose
    enum class pixelParkStates {
        INIT,
        SPIKE_NAV,
        BACKDROP,
        SHIFT,
        END_POSE,
        RETRACT,
        STOP,
    }

    enum class cycleStates {
        INIT,
        SPIKE_NAV,
        BACKDROP,

        //        SHIFT,
        RETRACT,
        PICK1,
        PLACE1,

        //        SHIFT2,
        END_POSE,
        RETRACT2,
        STOP
    }
}
