package org.firstinspires.ftc.teamcode.opModes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.customHardware.AutoHardware
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.StartSide
import org.firstinspires.ftc.teamcode.customHardware.camera.camUtil.PROCESSORS

class strippedAuto(private var alliance: Alliance, private var  startSide: StartSide) : LinearOpMode() {
    private lateinit var robot: AutoHardware

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        robot =
            AutoHardware(
                this,
                PROCESSORS.OBJECT_DETECT,
                StartLocation(alliance, startSide)
            )
        if (opModeIsActive()) {

        }
    }
}