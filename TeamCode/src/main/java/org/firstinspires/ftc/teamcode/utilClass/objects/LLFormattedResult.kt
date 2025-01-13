package org.firstinspires.ftc.teamcode.utilClass.objects

data class LLFormattedResult(
    val angle: Double,
    val centerX: Double,
    val centerY: Double,
    val color: BinaryArray,

    ) {
    companion object {
        fun empty(): LLFormattedResult {
            return LLFormattedResult(0.0, 0.0, 0.0, BinaryArray(2))
        }
    }
}