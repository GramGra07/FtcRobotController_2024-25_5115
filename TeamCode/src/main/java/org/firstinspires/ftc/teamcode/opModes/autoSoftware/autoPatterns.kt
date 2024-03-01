package org.firstinspires.ftc.teamcode.opModes.autoSoftware

import org.firstinspires.ftc.teamcode.Enums.EndPose
import org.firstinspires.ftc.teamcode.Enums.PathLong
import org.firstinspires.ftc.teamcode.Enums.StartDist
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.ExtendoSubsystem
import org.gentrifiedApps.statemachineftc.StateMachine

object autoPatterns {
    fun place1Machine(
        clawSubsystem: ClawSubsystem,
        driveSubsystem: DriveSubsystem,
        drive: MecanumDrive = driveSubsystem.drive!!
    ): StateMachine<place1States> {
        val builder = StateMachine.Builder<place1States>()
        return builder
            .state(place1States.SPIKE_NAV)
            .onEnter(place1States.SPIKE_NAV) {
                clawSubsystem.flipZero()
                clawSubsystem.update()
//                flipServo.calcFlipPose(0.0)
                generalPatterns.SpikeNav(drive, PathLong.NONE)
            }
            .whileState(
                place1States.SPIKE_NAV,
                { !drive.isBusy },

                { drive.update() })
            .onExit(
                place1States.SPIKE_NAV
            ) { //openClaw(claw2)
                clawSubsystem.openRight()
                clawSubsystem.update()
            }
            .transition(
                place1States.SPIKE_NAV,
                { !drive.isBusy }, 0.0
            )
            .state(place1States.END_POSE)
            .onEnter(
                place1States.END_POSE
            ) {
                clawSubsystem.flipBack()
                clawSubsystem.update()
//                flipServo.calcFlipPose(30.0)
                endPose.goToEndPose(EndPose.StartingPosition, drive)
            }
            .whileState(
                place1States.END_POSE,
                { !drive.isBusy },

                { drive.update() })
            .transition(
                place1States.END_POSE,
                { !drive.isBusy }, 0.0
            )
            .stopRunning(place1States.STOP)
            .build()
    }

    var rotate = 0
    var extend = 0
    fun pixelParkMachine(
        pathLong: PathLong,
        endPose: EndPose,
        clawSubsystem: ClawSubsystem,
        extendoSubsystem: ExtendoSubsystem,
        driveSubsystem: DriveSubsystem,
        drive: MecanumDrive = driveSubsystem.drive!!
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
                clawSubsystem.flipZero()
                clawSubsystem.update()
//                flipServo.calcFlipPose(0.0)
                generalPatterns.SpikeNav(drive, pathLong)
            }
            .whileState(
                pixelParkStates.SPIKE_NAV,
                { !drive.isBusy },
                { drive.update() })
            .onExit(pixelParkStates.SPIKE_NAV) {
                clawSubsystem.openRight()
                clawSubsystem.flipBack()
                clawSubsystem.update()
//                ServoUtil.openClaw(HardwareConfig.claw2)
//                flipServo.calcFlipPose(30.0)
            }
            .transition(pixelParkStates.SPIKE_NAV, { !drive.isBusy }, 0.0)
            .state(pixelParkStates.BACKDROP)
            .onEnter(pixelParkStates.BACKDROP) {
                clawSubsystem.flipBack()
                clawSubsystem.update()
//                flipServo.calcFlipPose(30.0)
                generalPatterns.navToBackdrop_Place(drive, pathLong, false, clawSubsystem)
            }
            .whileState(
                pixelParkStates.BACKDROP,
                { !drive.isBusy },
                { drive.update() })
            .onExit(pixelParkStates.BACKDROP) {
                if (HardwareConfig.Companion.startDist == StartDist.LONG_SIDE) {
                    rotate = extendoSubsystem.autoRotation - 400
                    extend = extendoSubsystem.autoExtension
                    //                        calculateFlipPose(AutoServoPositions.flipDown - clawOffset, flipServo);

                    extendoSubsystem.autoRotate(rotate, driveSubsystem)

//                    AutoHardware.encoderDrive(
//                        HardwareConfig.Companion.motorRotation,
//                        rotate,
//                        1.0,
//                        drive
//                    )
                } else {
                    extend = extendoSubsystem.autoExtension / 2
                }
                clawSubsystem.flipDown()
                clawSubsystem.update()
//                flipServo.calcFlipPose((flipDown - clawOffset).toDouble())
                extendoSubsystem.autoExtend(extend, driveSubsystem)
//                AutoHardware.encoderDrive(
//                    HardwareConfig.Companion.motorExtension,
//                    extend,
//                    1.0,
//                    drive
//                )
                clawSubsystem.openRight()
                clawSubsystem.update()
//                ServoUtil.openClaw(HardwareConfig.Companion.claw1)
            }
            .transition(pixelParkStates.BACKDROP, { !drive.isBusy }, 0.0)
            .state(pixelParkStates.END_POSE)
            .onEnter(pixelParkStates.END_POSE) {
                extendoSubsystem.autoExtend(-extend, driveSubsystem)
//                AutoHardware.encoderDrive(
//                    HardwareConfig.Companion.motorExtension,
//                    -extend,
//                    0.5,
//                    drive
//                )
//                flipServo.calcFlipPose(60.0)
                clawSubsystem.flipHigh()
                clawSubsystem.update()
                if (endPose != EndPose.NONE) {
                    org.firstinspires.ftc.teamcode.opModes.autoSoftware.endPose.goToEndPose(
                        endPose,
                        drive
                    )
                }
            }
            .whileState(
                pixelParkStates.END_POSE,
                { !drive.isBusy },
                { drive.update() })
            .transition(pixelParkStates.END_POSE, { !drive.isBusy }, 0.0)
            .state(pixelParkStates.RETRACT)
            .onEnter(pixelParkStates.RETRACT) {
                if (HardwareConfig.Companion.startDist == StartDist.LONG_SIDE) {
                    extendoSubsystem.autoRotate(-rotate, driveSubsystem)
//                    AutoHardware.encoderDrive(
//                        HardwareConfig.Companion.motorRotation,
//                        -rotate,
//                        1.0,
//                        drive
//                    )
                }
            }
            .onExit(pixelParkStates.RETRACT) {}
            .transition(pixelParkStates.RETRACT, { !drive.isBusy }, 0.0)
            .stopRunning(pixelParkStates.STOP)
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
        newState
    }
}
