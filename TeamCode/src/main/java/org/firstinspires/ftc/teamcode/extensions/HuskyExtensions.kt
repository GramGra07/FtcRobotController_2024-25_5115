package org.firstinspires.ftc.teamcode.extensions
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap

object HuskyExtensions{
    fun initHusky(val hw:hardwareMap, val name:string, val algorithm:HuskyLens.Algorithm):HuskyLens{
        val t = hw.get(HuskyLens::class.java,name)
        if (!huskyLens.knock()) {
            throw Exception("Cannot access HuskyLens $name")
            }
            t.selectAlgorithm(algorithm);
    }
    fun HuskyLens.getData():HuskyLens.Block[]{
HuskyLens.Block[] blocks = this.blocks();
return blocks
 }
 fun HuskyLens.telemetry(val telemetry:Telemetry){
 val blocks = this.getData()
val count = blocks.length 
telemetry.addData("Count",count)
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
            }
 }
}