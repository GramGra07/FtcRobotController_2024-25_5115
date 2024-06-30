package org.firstinspires.ftc.teamcode.opModes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.customHardware.AutoHardware
import org.firstinspires.ftc.teamcode.customHardware.autoHardware.StartLocation
import org.firstinspires.ftc.teamcode.customHardware.camera.camUtil.Processor
import org.firstinspires.ftc.teamcode.startEnums.Alliance
import org.firstinspires.ftc.teamcode.startEnums.StartSide
import org.firstinspires.ftc.teamcode.storage.PoseStorage

@Autonomous
class testAuto :LinearOpMode(){
    private lateinit var robot: AutoHardware

    override fun runOpMode() {
        robot =
            AutoHardware(
                this,
                hardwareMap,
                null,
                StartLocation(Alliance.RED, StartSide.LEFT)
            )
        if (opModeIsActive()) {
            PoseStorage.currentPose = robot.drive.pose
        }
    }
}