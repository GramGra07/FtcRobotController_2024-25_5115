package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance;
import org.firstinspires.ftc.teamcode.opModes.tuners.startupTest;
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles;

import java.util.ArrayList;
import java.util.List;

public final class TeleRegistrar {
    private static final List<Class<? extends OpMode>> opModeClasses = new ArrayList<>();

    static {
        opModeClasses.add(teleOp.class);
        opModeClasses.add(startupTest.class);
    }

    private TeleRegistrar() {
    }

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls, Alliance alliance) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName() + alliance.name())
                .setGroup(GroupingTitles.tele)
                .setFlavor(OpModeMeta.Flavor.TELEOP)

                .build();
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        for (Class<? extends OpMode> opModeClass : opModeClasses) {
            if (opModeClass == startupTest.class){
                manager.register(
                        metaForClass(opModeClass, Alliance.RED),
                        createInstance(opModeClass, Alliance.RED)
                );
                continue;
            }else {
                for (Alliance alliance : Alliance.values()) {
                    manager.register(
                            metaForClass(opModeClass, alliance),
                            createInstance(opModeClass, alliance)
                    );
                }
            }
        }
    }

    private static OpMode createInstance(Class<? extends OpMode> cls, Alliance alliance) {
        try {
            return cls.getConstructor(Alliance.class).newInstance(alliance);
        } catch (Exception e) {
            throw new RuntimeException("Failed to create instance of " + cls.getSimpleName(), e);
        }
    }
}