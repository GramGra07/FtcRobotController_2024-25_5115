package org.firstinspires.ftc.teamcode.customHardware.sensorArray

import org.firstinspires.ftc.robotcore.external.Telemetry


//ex

//val sensorArray: SensorArray = SensorArray()
//sensorArray.addSensor(Pair("axon", Sensor(SensorType.ENC, { axonServo = AxonServo(ahwMap, "airplaneRotation", 90.0)}, 1)))
class SensorArray {
    private lateinit var array: MutableList<Pair<String, Sensor>>
    fun addSensor(pair: Pair<String, Sensor>) {
        array.add(pair)
        pair.second.initializeSensor()
    }

    fun readAll() {
        array.forEach { sensor -> sensor.second.read() }
    }

    fun allTelemetry(telemetry: Telemetry) {
        array.forEach { sensor -> telemetry(telemetry, sensor.first) }
    }

    fun telemetry(telemetry: Telemetry, name: String) {
        val index = array.indexOf(array.find { it.first == name })
        telemetry.addData(array[index].first, array[index].second.value)
    }

    fun read(name: String) {
        array[array.indexOf(array.find { it.first == name })].second.read()
    }

    fun autoLoop(loops: Int) {
        array.forEach { sensor ->
            if (loops % sensor.second.period == 0) {
                sensor.second.read()
            }
        }
    }
}