package org.firstinspires.ftc.teamcode.customHardware.autoUtil.startEnums

import org.firstinspires.ftc.teamcode.utilClass.objects.BinaryArray

enum class Alliance {
    RED, BLUE;

    fun toChar(): String {

        return when (this) {
            Alliance.RED -> "R"
            Alliance.BLUE -> "B"
        }
    }
}

fun Alliance.toBinary(): BinaryArray {
    return when (this) {
        Alliance.RED -> BinaryArray(1).apply {
            this[0] = 0.0
        }

        Alliance.BLUE -> BinaryArray(1).apply {
            this[0] = 1.0
        }
    }
}

fun Alliance.toBinary2(): BinaryArray {
    return when (this) {
        Alliance.RED -> BinaryArray(2).apply {
            this[0] = 0.0
            this[1] = 1.0
        }

        Alliance.BLUE -> BinaryArray(2).apply {
            this[0] = 1.0
            this[1] = 0.0
        }
    }
}

fun BinaryArray.toAlliance(): Alliance {
    return when (this[0]) {
        0.0 -> Alliance.RED
        1.0 -> Alliance.BLUE
        else -> Alliance.RED
    }
}