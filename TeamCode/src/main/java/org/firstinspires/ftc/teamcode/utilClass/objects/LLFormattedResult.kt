package org.firstinspires.ftc.teamcode.utilClass.objects

data class LLFormattedResult(
    var angle: Double,
    var centerX: Double,
    var centerY: Double,
    var color: BinaryArray,

    ) {
    companion object {
        fun empty(): LLFormattedResult {
            return LLFormattedResult(0.0, 0.0, 0.0, BinaryArray(2))
        }
    }
}