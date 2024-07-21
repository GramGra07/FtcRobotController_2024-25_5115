package org.firstinspires.ftc.teamcode.ggutil.blank

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.customHardware.AutoHardware
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.StartSide
import org.firstinspires.ftc.teamcode.customHardware.camera.camUtil.Processor
import org.firstinspires.ftc.teamcode.storage.PoseStorage

@Autonomous
@Disabled
class blankAuto : LinearOpMode() {
    private lateinit var robot: AutoHardware

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        robot =
            AutoHardware(
                this,
                Processor.OBJECT_DETECT,
                StartLocation(Alliance.RED, StartSide.LEFT)
            )
        if (opModeIsActive()) {
            PoseStorage.currentPose = robot.localizerSubsystem.pose()
        }
    }
}