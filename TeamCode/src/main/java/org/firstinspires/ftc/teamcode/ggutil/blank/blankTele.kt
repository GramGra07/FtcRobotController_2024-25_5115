package org.firstinspires.ftc.teamcode.ggutil.blank

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.EndgameSubsystem
import org.firstinspires.ftc.teamcode.subsystems.ExtendoSubsystem

@TeleOp
@Disabled
class blankTele : LinearOpMode() {
    var robot = HardwareConfig(this)

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        HardwareConfig.init(hardwareMap, false)
        waitForStart()
        while (opModeIsActive()) { //while the op mode is active
            robot.doBulk()
        }
    }
}