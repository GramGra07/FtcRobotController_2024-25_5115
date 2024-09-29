package org.firstinspires.ftc.teamcode.opModes.tuners.OTOS

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.storage.GameStorage
import org.firstinspires.ftc.teamcode.storage.PoseStorage
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles

@TeleOp(group = GroupingTitles.TESTING)
//@Disabled//disabling the opmode
//@Config
class OTOSLinearScalarTuner : LinearOpMode() {
    //declaring the class
    var distance = 30
    private var deltaDistance = 0.0
    override fun runOpMode() { //if opmode is started
        val robot = HardwareConfig(this, false, StartLocation(GameStorage.alliance, PoseStorage.currentPose))
        telemetry.addData(
            "Move the robot forward 30 inches",
            "It will tell you the correct linear scalar"
        )
        robot.localizerSubsystem.update(null)
        val startY = robot.localizerSubsystem.x()
        val targetY = startY + distance
        telemetry.update()
        waitForStart()
        while (opModeIsActive()) {
            robot.localizerSubsystem.update(null)
            deltaDistance = robot.localizerSubsystem.y() + startY
            telemetry.addData("traveled",robot.localizerSubsystem.x())
            telemetry.addData("deltaDistance", deltaDistance)
            telemetry.addData("targetY", targetY)
            telemetry.addData("Linear scalar", targetY / deltaDistance)
            telemetry.update()
        }
    }
}