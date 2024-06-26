package org.firstinspires.ftc.teamcode.customHardware.sensorArray

import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.customHardware.sensors.BeamBreakSensor

class Sensor(private var type: SensorType, private var initializer: Runnable, var period: Int) {
    private lateinit var enc: DcMotorEx
    private lateinit var dist: DistanceSensor
    private lateinit var color: ColorSensor
    private lateinit var touch: TouchSensor
    private lateinit var beamBreak: BeamBreakSensor
    fun read(): Any {
        return when (type) {
            SensorType.ENC -> enc.currentPosition

            SensorType.DIST -> dist.getDistance(DistanceUnit.INCH)

            SensorType.COLOR -> color.argb()

            SensorType.TOUCH -> touch.isPressed

            SensorType.BB -> beamBreak.isBroken()
        }
    }

    fun initializeSensor() {
        initializer.run()
    }
}