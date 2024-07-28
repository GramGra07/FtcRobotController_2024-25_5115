package org.firstinspires.ftc.teamcode.opModes.testers

import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.customHardware.HardwareConfig
import org.firstinspires.ftc.teamcode.utilClass.GroupingTitles
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.varConfig.OTOShOffset
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.varConfig.OTOSxOffset
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.varConfig.OTOSyOffset

@TeleOp(group = GroupingTitles.TESTING)
//@Disabled//disabling the opmode
//@Config
class OTOSTuner : LinearOpMode() {
    //declaring the class

    override fun runOpMode() { //if opmode is started
        val robot = HardwareConfig(this, false)
        OTOSxOffset = robot.sparkFunOTOS.offset.x
        OTOSyOffset = robot.sparkFunOTOS.offset.y
        OTOShOffset = robot.sparkFunOTOS.offset.h
        waitForStart()
        robot.once() //runs once
        while (opModeIsActive()) {
            robot.sparkFunOTOS.offset = Pose2D(OTOSxOffset, OTOSyOffset, OTOShOffset)
            robot.doBulk()
        }
    }
}