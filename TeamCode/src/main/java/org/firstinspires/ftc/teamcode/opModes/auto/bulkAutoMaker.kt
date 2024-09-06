package org.firstinspires.ftc.teamcode.opModes.auto


import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.StartSide
import org.firstinspires.ftc.teamcode.followers.pedroPathing.localization.tuning.FowardTuner
import org.firstinspires.ftc.teamcode.followers.pedroPathing.localization.tuning.LateralTuner
import org.firstinspires.ftc.teamcode.followers.pedroPathing.localization.tuning.LocalizationTest
import org.firstinspires.ftc.teamcode.followers.pedroPathing.localization.tuning.TurnTuner
import org.firstinspires.ftc.teamcode.followers.pedroPathing.tuning.Circle
import org.firstinspires.ftc.teamcode.followers.pedroPathing.tuning.CurvedBackAndForth
import org.firstinspires.ftc.teamcode.followers.pedroPathing.tuning.ForwardVelocityTuner
import org.firstinspires.ftc.teamcode.followers.pedroPathing.tuning.ForwardZeroPowerAccelerationTuner
import org.firstinspires.ftc.teamcode.followers.pedroPathing.tuning.LateralZeroPowerAccelerationTuner
import org.firstinspires.ftc.teamcode.followers.pedroPathing.tuning.StrafeVelocityTuner
import org.firstinspires.ftc.teamcode.followers.pedroPathing.tuning.StraightBackAndForth
import org.firstinspires.ftc.teamcode.opModes.tuners.OTOS.J5155OTOSAngularScalar
import org.firstinspires.ftc.teamcode.opModes.tuners.OTOS.J5155OTOSHeadingOffsetTuner
import org.firstinspires.ftc.teamcode.opModes.tuners.OTOS.J5155OTOSPositionOffsetTuner
import org.firstinspires.ftc.teamcode.opModes.tuners.OTOS.OTOSAngularScalarTuner
import org.firstinspires.ftc.teamcode.opModes.tuners.OTOS.OTOSLinearScalarTuner
import org.firstinspires.ftc.teamcode.opModes.tuners.OTOS.OTOSOffsetAutoTuner
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles

object bulkAutoMaker {

    private val allOptionsParams: Pair<String, Boolean> = Pair(GroupingTitles.auto, false)
    private val alliances: List<Alliance> = listOf(Alliance.RED, Alliance.BLUE)
    private val startSides: List<StartSide> = listOf(StartSide.LEFT, StartSide.RIGHT)

    private val allOptions: Pair<List<Alliance>, List<StartSide>> = Pair(alliances, startSides)

    private val pedroParams: Pair<String, Boolean> = Pair(GroupingTitles.pedroTuning, false)
    private val pedroTuners: List<Pair<Class<out OpMode>, OpModeMeta.Flavor>> = listOf(
        Pair(Circle::class.java, OpModeMeta.Flavor.AUTONOMOUS),
        Pair(CurvedBackAndForth::class.java, OpModeMeta.Flavor.AUTONOMOUS),
        Pair(ForwardVelocityTuner::class.java, OpModeMeta.Flavor.AUTONOMOUS),
        Pair(ForwardZeroPowerAccelerationTuner::class.java, OpModeMeta.Flavor.AUTONOMOUS),
        Pair(LateralZeroPowerAccelerationTuner::class.java, OpModeMeta.Flavor.AUTONOMOUS),
        Pair(StrafeVelocityTuner::class.java, OpModeMeta.Flavor.AUTONOMOUS),
        Pair(StraightBackAndForth::class.java, OpModeMeta.Flavor.AUTONOMOUS),
        Pair(LateralTuner::class.java, OpModeMeta.Flavor.AUTONOMOUS),
        Pair(FowardTuner::class.java, OpModeMeta.Flavor.AUTONOMOUS),
        Pair(LocalizationTest::class.java, OpModeMeta.Flavor.AUTONOMOUS),
        Pair(TurnTuner::class.java, OpModeMeta.Flavor.AUTONOMOUS)
    )

    private val otosParams: Pair<String, Boolean> = Pair(GroupingTitles.pedroTuning, false)
    private val otosTuners: List<Pair<Class<out OpMode>, OpModeMeta.Flavor>> = listOf(
        Pair(J5155OTOSAngularScalar::class.java, OpModeMeta.Flavor.TELEOP),
        Pair(J5155OTOSHeadingOffsetTuner::class.java, OpModeMeta.Flavor.TELEOP),
        Pair(J5155OTOSPositionOffsetTuner::class.java, OpModeMeta.Flavor.TELEOP),
        Pair(OTOSAngularScalarTuner::class.java, OpModeMeta.Flavor.TELEOP),
        Pair(OTOSLinearScalarTuner::class.java, OpModeMeta.Flavor.TELEOP),
        Pair(OTOSOffsetAutoTuner::class.java, OpModeMeta.Flavor.TELEOP),
    )

    private fun metaForClass(name: String, group: String, flavor: OpModeMeta.Flavor): OpModeMeta {
        return OpModeMeta.Builder()
            .setName(name)
            .setGroup(group)
            .setFlavor(flavor)
            .build()
    }

    private fun buildName(options: Pair<Alliance, StartSide>): String {
        return "${options.first.toString()[0]}${options.second.toString()[0]}Auto"
    }

    @OpModeRegistrar
    fun register(manager: OpModeManager) {
        if (!allOptionsParams.second) {
            allOptions.first.forEach { alliance ->
                allOptions.second.forEach { startSide ->
                    val name = buildName(Pair(alliance, startSide))
                    manager.register(
                        metaForClass(name, allOptionsParams.first, OpModeMeta.Flavor.AUTONOMOUS),
                        strippedAuto(alliance, startSide)
                    )
                }
            }
        }
        if (!pedroParams.second) {
            pedroTuners.forEach { tuner ->
                manager.register(
                    metaForClass(
                        tuner.first.simpleName,
                        pedroParams.first,
                        tuner.second
                    ), tuner.first
                )
            }
        }
        if (!otosParams.second) {
            otosTuners.forEach { tuner ->
                manager.register(
                    metaForClass(
                        tuner.first.simpleName,
                        otosParams.first,
                        tuner.second
                    ), tuner.first
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