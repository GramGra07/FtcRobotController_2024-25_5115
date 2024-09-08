package org.firstinspires.ftc.teamcode.opModes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance;
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.StartSide;
import org.firstinspires.ftc.teamcode.followers.pedroPathing.localization.tuning.FowardTuner;
import org.firstinspires.ftc.teamcode.followers.pedroPathing.localization.tuning.LateralTuner;
import org.firstinspires.ftc.teamcode.followers.pedroPathing.localization.tuning.LocalizationTest;
import org.firstinspires.ftc.teamcode.followers.pedroPathing.localization.tuning.TurnTuner;
import org.firstinspires.ftc.teamcode.followers.pedroPathing.tuning.Circle;
import org.firstinspires.ftc.teamcode.followers.pedroPathing.tuning.CurvedBackAndForth;
import org.firstinspires.ftc.teamcode.followers.pedroPathing.tuning.ForwardVelocityTuner;
import org.firstinspires.ftc.teamcode.followers.pedroPathing.tuning.ForwardZeroPowerAccelerationTuner;
import org.firstinspires.ftc.teamcode.followers.pedroPathing.tuning.LateralZeroPowerAccelerationTuner;
import org.firstinspires.ftc.teamcode.followers.pedroPathing.tuning.StrafeVelocityTuner;
import org.firstinspires.ftc.teamcode.followers.pedroPathing.tuning.StraightBackAndForth;
import org.firstinspires.ftc.teamcode.opModes.tuners.OTOS.J5155OTOSAngularScalar;
import org.firstinspires.ftc.teamcode.opModes.tuners.OTOS.J5155OTOSHeadingOffsetTuner;
import org.firstinspires.ftc.teamcode.opModes.tuners.OTOS.J5155OTOSPositionOffsetTuner;
import org.firstinspires.ftc.teamcode.opModes.tuners.OTOS.OTOSAngularScalarTuner;
import org.firstinspires.ftc.teamcode.opModes.tuners.OTOS.OTOSLinearScalarTuner;
import org.firstinspires.ftc.teamcode.opModes.tuners.OTOS.OTOSOffsetAutoTuner;
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public final class bulkAutoMaker {

    private final Pair<String, Boolean> allOptionsParams = new Pair<>(GroupingTitles.auto, false);
    private final List<Alliance> alliances = Arrays.asList(Alliance.RED, Alliance.BLUE);
    private final List<StartSide> startSides = Arrays.asList(StartSide.LEFT, StartSide.RIGHT);

    private final Pair<List<Alliance>, List<StartSide>> allOptions = new Pair<>(alliances, startSides);

    private final Pair<String, Boolean> pedroParams = new Pair<>(GroupingTitles.pedroTuning, false);
    private final List<Pair<Class<? extends OpMode>, OpModeMeta.Flavor>> pedroTuners = Arrays.asList(
            new Pair<>(Circle.class, OpModeMeta.Flavor.AUTONOMOUS),
            new Pair<>(CurvedBackAndForth.class, OpModeMeta.Flavor.AUTONOMOUS),
            new Pair<>(ForwardVelocityTuner.class, OpModeMeta.Flavor.AUTONOMOUS),
            new Pair<>(ForwardZeroPowerAccelerationTuner.class, OpModeMeta.Flavor.AUTONOMOUS),
            new Pair<>(LateralZeroPowerAccelerationTuner.class, OpModeMeta.Flavor.AUTONOMOUS),
            new Pair<>(StrafeVelocityTuner.class, OpModeMeta.Flavor.AUTONOMOUS),
            new Pair<>(StraightBackAndForth.class, OpModeMeta.Flavor.AUTONOMOUS),
            new Pair<>(LateralTuner.class, OpModeMeta.Flavor.AUTONOMOUS),
            new Pair<>(FowardTuner.class, OpModeMeta.Flavor.AUTONOMOUS),
            new Pair<>(LocalizationTest.class, OpModeMeta.Flavor.AUTONOMOUS),
            new Pair<>(TurnTuner.class, OpModeMeta.Flavor.AUTONOMOUS)
    );

    private final Pair<String, Boolean> otosParams = new Pair<>(GroupingTitles.pedroTuning, false);
    private final List<Pair<Class<? extends OpMode>, OpModeMeta.Flavor>> otosTuners = Arrays.asList(
            new Pair<>(J5155OTOSAngularScalar.class, OpModeMeta.Flavor.TELEOP),
            new Pair<>(J5155OTOSHeadingOffsetTuner.class, OpModeMeta.Flavor.TELEOP),
            new Pair<>(J5155OTOSPositionOffsetTuner.class, OpModeMeta.Flavor.TELEOP),
            new Pair<>(OTOSAngularScalarTuner.class, OpModeMeta.Flavor.TELEOP),
            new Pair<>(OTOSLinearScalarTuner.class, OpModeMeta.Flavor.TELEOP),
            new Pair<>(OTOSOffsetAutoTuner.class, OpModeMeta.Flavor.TELEOP)
    );

    private OpModeMeta metaForClass(String name, String group, OpModeMeta.Flavor flavor) {
        return new OpModeMeta.Builder()
                .setName(name)
                .setGroup(group)
                .setFlavor(flavor)
                .build();
    }

    private String buildName(Pair<Alliance, StartSide> options) {
        return options.first.toString().substring(0, 1) + options.second.toString().substring(0, 1) + "Auto";
    }

    @OpModeRegistrar
    public void register(OpModeManager manager) {
        if (!allOptionsParams.second) {
            for (Alliance alliance : allOptions.first) {
                for (StartSide startSide : allOptions.second) {
                    String name = buildName(new Pair<>(alliance, startSide));
                    manager.register(
                            metaForClass(name, allOptionsParams.first, OpModeMeta.Flavor.AUTONOMOUS),
                            new strippedAuto(alliance, startSide)
                    );
                }
            }
        }

        if (!pedroParams.second) {
            for (Pair<Class<? extends OpMode>, OpModeMeta.Flavor> tuner : pedroTuners) {
                manager.register(
                        metaForClass(
                                tuner.first.getSimpleName(),
                                pedroParams.first,
                                tuner.second
                        ),
                        tuner.first
                );
            }
        }

        if (!otosParams.second) {
            for (Pair<Class<? extends OpMode>, OpModeMeta.Flavor> tuner : otosTuners) {
                manager.register(
                        metaForClass(
                                tuner.first.getSimpleName(),
                                otosParams.first,
                                tuner.second
                        ),
                        tuner.first
                );
            }
        }
        FtcDashboard.getInstance().withConfigRoot(configRoot -> {
            for (Class<?> c : Arrays.asList(
                    Circle.class
            )) {
                configRoot.putVariable(c.getSimpleName(), ReflectionConfig.createVariableFromClass(c));
            }
        });
    }

    // Inner Pair class to replicate Kotlin's Pair functionality
    private static class Pair<F, S> {
        private final F first;
        private final S second;

        public Pair(F first, S second) {
            this.first = first;
            this.second = second;
        }

        public F getFirst() {
            return first;
        }

        public S getSecond() {
            return second;
        }
    }
}