//import
package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.extensions.BlinkExtensions.initLights
import org.firstinspires.ftc.teamcode.extensions.BlinkExtensions.setPatternCo
import org.firstinspires.ftc.teamcode.extensions.SensorExtensions.initVSensor


open class HardwareConfig() {

    constructor(opMode: LinearOpMode, ahwMap: HardwareMap) : this() {
        myOpMode = opMode
        this.initRobot(ahwMap)
    }

    companion object {
        lateinit var lights: RevBlinkinLedDriver
        lateinit var vSensor: VoltageSensor
        private lateinit var myOpMode: LinearOpMode

        var allHubs: List<LynxModule> = ArrayList()
    }

    private fun initRobot(
        ahwMap: HardwareMap,
    ) {
        vSensor = initVSensor(ahwMap, "Expansion Hub 2")
        lights = initLights(ahwMap, "blinkin")
        allHubs = ahwMap.getAll(LynxModule::class.java)
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }
        lights.setPatternCo(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE)
    }

    fun doBulk() {
        if (myOpMode.gamepad1.cross) {
            lights.setPatternCo(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE)
        } else if (myOpMode.gamepad1.circle) {
            lights.setPatternCo(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE)
        }
    }
}

