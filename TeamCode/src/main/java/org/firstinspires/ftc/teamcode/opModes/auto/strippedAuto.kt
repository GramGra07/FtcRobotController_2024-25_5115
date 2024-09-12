package org.firstinspires.ftc.teamcode.opModes.auto

import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.customHardware.AutoHardware
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.StartSide

class strippedAuto(private var alliance: Alliance,) :
    LinearOpMode() {
    private lateinit var robot: AutoHardware

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        robot =
            AutoHardware(
                this,
                StartLocation(alliance, Pose2d(0.0, 0.0, -Math.PI / 2))
            )
        if (opModeIsActive()) {

        }
    }
}