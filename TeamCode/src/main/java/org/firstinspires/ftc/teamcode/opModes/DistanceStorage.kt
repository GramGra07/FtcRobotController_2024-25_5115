package org.firstinspires.ftc.teamcode.opModes

data class DistanceStorage(val totalDist: Double = 0.0) {
    companion object {
        var totalDist: Double = 0.0
    }

    fun getDist(): Double = totalDist
}
