package org.firstinspires.ftc.teamcode.utilClass.drivetrain

data class DrivePARAMS(
    // drive model parameters
    var inPerTick: Double,
    var lateralInPerTick: Double,
    var trackWidthTicks: Double,

    // feedforward parameters (in tick units)
    var kS: Double,
    var kV: Double,
    var kA: Double,

    // path profile parameters (in inches)
    var maxWheelVel: Double,
    var minProfileAccel: Double,
    var maxProfileAccel: Double,

    // turn profile parameters (in radians)
    var maxAngVel: Double, // shared with path
    var maxAngAccel: Double,

    // path controller gains
    var axialGain: Double,
    var lateralGain: Double,
    var headingGain: Double, // shared with turn

    var axialVelGain: Double,
    var lateralVelGain: Double,
    var headingVelGain: Double // shared with turn
)