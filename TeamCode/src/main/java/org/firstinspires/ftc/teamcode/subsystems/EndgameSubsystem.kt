package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Limits.liftMax
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.releaseAirplane
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.resetAirplane
import org.firstinspires.ftc.teamcode.extensions.MotorExtensions.initMotor
import org.firstinspires.ftc.teamcode.extensions.ServoExtensions.initServo

class EndgameSubsystem(ahwMap: HardwareMap) {
    enum class ShooterState {


        OPEN,
        CLOSED
    }

    enum class LiftStates {
        RETRACT,
        EXTEND,
        STOP
    }
    private var shootState:ShooterState = ShooterState.CLOSED
    private var liftState:LiftStates = LiftStates.STOP

    private var motorLift: DcMotor? = null

    private var airplaneServo: Servo? = null

    var planeReleased = false


    init {
        if (motorLift == null) {
            motorLift = initMotor(
                ahwMap,
                "lift",
                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                DcMotorSimple.Direction.REVERSE
            )
        }
        if (airplaneServo == null) {
            airplaneServo = initServo(ahwMap, "airplaneServo")
        }
    }
    fun retract(){
        liftState = LiftStates.RETRACT
    }
    fun extend(){
        liftState = LiftStates.EXTEND
    }
    fun stopLift(){
        liftState = LiftStates.STOP
    }
    fun shoot(){
        shootState = ShooterState.OPEN
    }
    fun resetAirplane(){
        shootState = ShooterState.CLOSED
    }
    private fun power(){
        when (liftState) {
            LiftStates.RETRACT -> {
                motorLift!!.power = liftMax
            }
            LiftStates.EXTEND -> {
                motorLift!!.power = -liftMax
            }
            LiftStates.STOP -> {
                motorLift!!.power = 0.0
            }
        }
    }

    fun update(){
        power()
        when (shootState) {
            ShooterState.OPEN -> {
                releaseAirplane(airplaneServo!!)
            }
            ShooterState.CLOSED -> {
                resetAirplane(airplaneServo!!)
            }
        }
    }
}