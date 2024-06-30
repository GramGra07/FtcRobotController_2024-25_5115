package org.firstinspires.ftc.teamcode.ggutil.blank

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.customHardware.AutoHardware
import org.firstinspires.ftc.teamcode.customHardware.autoHardware.StartLocation
import org.firstinspires.ftc.teamcode.startEnums.Alliance
import org.firstinspires.ftc.teamcode.startEnums.StartSide
import org.firstinspires.ftc.teamcode.storage.PoseStorage
import org.firstinspires.ftc.teamcode.customHardware.camera.camUtil.Processor

@Autonomous
@Disabled
class blankAuto : LinearOpMode() {
    private lateinit var robot: AutoHardware

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        robot =
            AutoHardware(
                this,
                hardwareMap,
                Processor.OBJECT_DETECT,
                StartLocation(Alliance.RED, StartSide.LEFT)
            )
        if (opModeIsActive()) {
            PoseStorage.currentPose = robot.drive.pose
        }
    }
}