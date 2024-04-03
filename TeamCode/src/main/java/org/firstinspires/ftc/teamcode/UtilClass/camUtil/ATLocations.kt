package org.firstinspires.ftc.teamcode.UtilClass.camUtil

import org.firstinspires.ftc.teamcode.Point

class ATLocations {
    companion object {
        private var allLocations: List<Pair<Int, AprilTagLocation>> =
            listOf(
                Pair(1, AprilTagLocation(Point(-36.0 - 4.8, 61.0))), //bb Left
                Pair(2, AprilTagLocation(Point(-36.0, 61.0))), //bb mid
                Pair(3, AprilTagLocation(Point(-36.0 + 4.8, 61.0))), //bb right
                Pair(4, AprilTagLocation(Point(36.0 - 4.8, 61.0))), //rb left
                Pair(5, AprilTagLocation(Point(36.0, 61.0))), //rb mid
                Pair(6, AprilTagLocation(Point(36.0 + 4.8, 61.0))), //rb right
                Pair(7, AprilTagLocation(Point(38.0, -72.0))), // big red audience
                Pair(8, AprilTagLocation(Point(35.0, -72.0))), // small red audience
                Pair(9, AprilTagLocation(Point(35.0, -72.0))), // small blue audience
                Pair(10, AprilTagLocation(Point(-38.0, -72.0))), // big blue audience
            )

        fun getLocation(id: Int): Point {
            return allLocations[id].location()
        }

        private fun Pair<Int, AprilTagLocation>.location(): Point {
            return this.second.location
        }
    }
}