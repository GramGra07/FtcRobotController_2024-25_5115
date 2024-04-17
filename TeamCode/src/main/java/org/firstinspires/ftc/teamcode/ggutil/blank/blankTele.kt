package org.firstinspires.ftc.teamcode.ggutil.blank

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig

@TeleOp
@Disabled
class blankTele : LinearOpMode() {
    var robot = HardwareConfig(this, hardwareMap, false)

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        waitForStart()
        robot.once(this)
        while (opModeIsActive()) { //while the op mode is active
            robot.doBulk()
        }
    }
}