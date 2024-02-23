package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config

@Config
object Limits {
    @JvmField
    var flipperMin = -0.7

    @JvmField
    var flipperMax = 0.7

    @JvmField
    var liftMax = 1.0

    @JvmField
    var liftMin = -0.7

    @JvmField
    var slideMax = 1.0

    @JvmField
    var slideMin = -1.0

    @JvmField
    var autoExtension = 1100

    @JvmField
    var autoRotation = 800

    @JvmField
    var maxExtensionTicks = 3280

    @JvmField
    var minExtensionTicks = 0

    @JvmField
    var maxRotationTicks = 1424

    @JvmField
    var maxPotent = 65

    @JvmField
    var minRotationTicks = 0
}
