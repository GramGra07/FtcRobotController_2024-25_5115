package org.firstinspires.ftc.teamcode.subsystems.gameSpecific

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.extensions.MotorExtensions.initMotor
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.initServo


class EndgameSubsystem(ahwMap: HardwareMap) {
    enum class ShooterState {
        OPEN,
        CLOSED,
        IDLE
    }

    enum class LiftStates {
        RETRACT,
        EXTEND,
        STOP
    }

    private var shootState: ShooterState = ShooterState.CLOSED
    private var liftState: LiftStates = LiftStates.STOP

    private var motorLift: DcMotor

    private var airplaneServo: Servo

    var planeReleased = false

    private var liftMax = 1.0


    init {
        motorLift = initMotor(
            ahwMap,
            "lift",
            DcMotor.RunMode.RUN_WITHOUT_ENCODER,
            DcMotorSimple.Direction.REVERSE
        )
        airplaneServo = initServo(ahwMap, "airplaneServo")

    }

    class endGameDefault : CommandBase() {
        private fun update() {
            this.update()
        }

        override fun execute() {
            update()
        }
    }

    fun retract() {
        liftState = LiftStates.RETRACT
    }

    fun extend() {
        liftState = LiftStates.EXTEND
    }

    fun stopLift() {
        liftState = LiftStates.STOP
    }

    fun shoot() {
        shootState = ShooterState.OPEN
    }

    fun resetAirplane() {
        shootState = ShooterState.CLOSED
    }

    private fun power() {
        when (liftState) {
            LiftStates.RETRACT -> {
                motorLift.power = liftMax
            }

            LiftStates.EXTEND -> {
                motorLift.power = -liftMax
            }

            LiftStates.STOP -> {
                motorLift.power = 0.0
            }
        }
    }

    fun update() {
        power()
//        when (shootState) {
//            ShooterState.OPEN -> {
//                releaseAirplane(airplaneServo)
//                shootState = ShooterState.IDLE
//            }
//
//            ShooterState.CLOSED -> {
//                resetAirplane(airplaneServo)
//                shootState = ShooterState.IDLE
//            }
//
//            ShooterState.IDLE -> {}
//        }
    }
}