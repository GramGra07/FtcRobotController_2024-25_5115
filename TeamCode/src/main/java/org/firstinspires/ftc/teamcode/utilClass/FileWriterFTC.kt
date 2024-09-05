package org.firstinspires.ftc.teamcode.utilClass

import android.os.Environment
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.VarConfig
import java.io.File
import java.io.FileWriter

object FileWriterFTC {
    val file = String.format(
        "%s/FIRST/matchlogs/log.txt",
        Environment.getExternalStorageDirectory().absolutePath
    )

    fun setUpFile(fileWriter: FileWriter) {
        if (VarConfig.useFileWriter) {
            val myObj = File(file)
            try {
                myObj.delete()
                myObj.createNewFile()
            } catch (e: Exception) {
                e.printStackTrace()
            }
            try {
                fileWriter.write("")
                fileWriter.close()
            } catch (e: Exception) {
                e.printStackTrace()
            }
        }
    }

    fun FileWriter.init() {
        val file = String.format(
            "%s/FIRST/matchlogs/log.txt",
            Environment.getExternalStorageDirectory().absolutePath
        )
    }

    fun writeToFile(fileWriter: FileWriter, x: Int, y: Int) {
        //  terminal
        //  clear
        //  adb shell
        //  cat /storage/emulated/0/FIRST/matchlogs/log.txt
        //  copy everything
        //  paste into file.txt
        //  run testGraphing in pycharm or other python IDE
        //  look at results in graph.png
        var fileWriter = fileWriter
        if (VarConfig.useFileWriter) {
            try {
                fileWriter = FileWriter(file, true)
                fileWriter.append(x.toString()).append(" ").append(y.toString()).append(" \n")
                fileWriter.close()
            } catch (e: Exception) {
                e.printStackTrace()
            }
        }
    }
}
