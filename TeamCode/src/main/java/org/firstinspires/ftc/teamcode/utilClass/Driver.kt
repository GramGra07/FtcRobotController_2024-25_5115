package org.firstinspires.ftc.teamcode.utilClass

import org.firstinspires.ftc.teamcode.subsystems.avoidance.AvoidanceSubsystem
import org.firstinspires.ftc.teamcode.subsystems.humanInput.Drivers
import org.firstinspires.ftc.teamcode.utilClass.objects.DriveType

class Driver(
    var name: Drivers.AllDrivers,
    var defaultAvoidance: AvoidanceSubsystem.AvoidanceTypes,
    var fieldCentric: DriveType
)