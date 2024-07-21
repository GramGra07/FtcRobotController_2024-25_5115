package org.firstinspires.ftc.teamcode.extensions

import com.qualcomm.hardware.dfrobot.HuskyLens
import com.qualcomm.hardware.dfrobot.HuskyLens.Block
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

object HuskyExtensions {
    fun initHusky(hw: HardwareMap, name: String, algorithm: HuskyLens.Algorithm): HuskyLens {
        val t = hw.get(HuskyLens::class.java, name)
        if (!t.knock()) {
            throw Exception("Cannot access HuskyLens $name")
        }
        t.selectAlgorithm(algorithm)
        return t
    }

    fun HuskyLens.getData(): Array<out Block>? {
        return this.blocks()
    }

    fun HuskyLens.telemetry(telemetry: Telemetry) {
        val blocks = this.getData()
        val count = blocks!!.size
        telemetry.addData("Count", count)
        for (i in blocks.indices) {
            telemetry.addData("Block", blocks[i].toString())
        }

    }
}