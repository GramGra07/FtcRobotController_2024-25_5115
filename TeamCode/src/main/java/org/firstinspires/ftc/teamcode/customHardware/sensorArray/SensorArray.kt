package org.firstinspires.ftc.teamcode.customHardware.sensorArray

import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig
import org.firstinspires.ftc.teamcode.customHardware.servos.AxonServo
import org.firstinspires.ftc.teamcode.subsystems.loopTime.LoopTimeController


//ex

//val sensorArray: SensorArray = SensorArray()
//sensorArray.addSensor(Pair("axon", Sensor(SensorType.ENC, { axonServo = AxonServo(ahwMap, "airplaneRotation", 90.0)}, 1)))
class SensorArray() {
    private lateinit var array: MutableList<Pair<String, Sensor>>
    fun addSensor(pair: Pair<String, Sensor>) {
        array.add(pair)
        pair.second.initializeSensor()
    }

    fun readAll() {
        array.forEach { sensor -> sensor.second.read() }
    }

    fun read(name: String) {
        array[array.indexOf(array.find { it.first == name })].second.read()
    }

    fun autoLoop(loops:Int) {
        array.forEach { sensor ->
            if (loops % sensor.second.period == 0) {
                sensor.second.read()
            }
        }
    }
}