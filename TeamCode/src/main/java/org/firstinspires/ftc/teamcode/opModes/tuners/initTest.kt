package org.firstinspires.ftc.teamcode.opModes.tuners

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.StartLocation
import org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums.Alliance
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles

//@TeleOp(group = GroupingTitles.tele)
class initTest : LinearOpMode() {
    lateinit var motorEx: List<DcMotorEx>
    override fun runOpMode() {
        motorEx = listOf(
            hardwareMap.get(DcMotorEx::class.java, "motorFrontRight"),
            hardwareMap.get(DcMotorEx::class.java, "extendMotor"),
            hardwareMap.get(DcMotorEx::class.java, "extendMotor2")
        )
        val robot = HardwareConfig(this, false, StartLocation(Alliance.RED))
        waitForStart()
        if (opModeIsActive()) {
            motorEx.forEach {
                it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                it.mode = DcMotor.RunMode.RUN_USING_ENCODER
            }
            robot.scoringSubsystem.setup()
            robot.scoringSubsystem.update()
            sleep(2000)
            robot.scoringSubsystem.setPitchMed()
            robot.scoringSubsystem.setRotateLeft()
            robot.scoringSubsystem.openClaw()
            robot.scoringSubsystem.update()
            sleep(2000)
            robot.scoringSubsystem.setup()
            robot.scoringSubsystem.update()
            sleep(2000)
        }
    }

}