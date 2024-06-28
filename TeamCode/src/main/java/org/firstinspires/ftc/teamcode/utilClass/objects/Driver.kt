package org.firstinspires.ftc.teamcode.utilClass.objects

import org.firstinspires.ftc.teamcode.subsystems.avoidance.AvoidanceSubsystem
import org.firstinspires.ftc.teamcode.subsystems.humanInput.Drivers

class Driver(
    var name: Drivers.AllDrivers,
    var defaultAvoidance: AvoidanceSubsystem.AvoidanceTypes,
    var fieldCentric: DriveType
)