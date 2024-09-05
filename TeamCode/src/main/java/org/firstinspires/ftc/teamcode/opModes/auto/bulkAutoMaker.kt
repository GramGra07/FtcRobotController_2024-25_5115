package org.firstinspires.ftc.teamcode.opModes.auto


import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig
import com.acmerobotics.dashboard.config.variable.CustomVariable
import com.acmerobotics.roadrunner.ftc.AngularRampLogger
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.StartSide
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles
import java.util.Arrays

object bulkAutoMaker {

    private const val GROUP: String = GroupingTitles.auto
    private const val DISABLED: Boolean = false

    private val alliances: List<Alliance> = listOf(Alliance.RED, Alliance.BLUE)
    private val startSides: List<StartSide> = listOf(StartSide.LEFT, StartSide.RIGHT)

    private val allOptions: Pair<List<Alliance>, List<StartSide>> = Pair(alliances, startSides)

    private fun metaForClass(name: String, cls: Class<out OpMode?>): OpModeMeta {
        return OpModeMeta.Builder()
            .setName(name)
            .setGroup(GROUP)
            .setFlavor(OpModeMeta.Flavor.AUTONOMOUS)
            .build()
    }

    private fun buildName(options: Pair<Alliance, StartSide>): String {
        return "${options.first.toString()[0]}${options.second.toString()[0]}Auto"
    }

    @OpModeRegistrar
    fun register(manager: OpModeManager) {
        if (DISABLED) return

        allOptions.first.forEach() { alliance ->
            allOptions.second.forEach(){ startSide ->
                val name = buildName(Pair(alliance, startSide))
                manager.register(
                    metaForClass(name, strippedAuto::class.java),
                    strippedAuto(alliance, startSide)
                )
            }
        }

//        FtcDashboard.getInstance().withConfigRoot()
//        { configRoot: CustomVariable ->
//            for (c in Arrays.asList<Class<out Any?>>(
//                AngularRampLogger::class.java,
//            )) {
//                configRoot.putVariable(
//                    c.simpleName,
//                    ReflectionConfig.createVariableFromClass(c)
//                )
//            }
//        }
    }
}