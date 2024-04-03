package org.firstinspires.ftc.teamcode.camera.setupClasses

class LensIntrinsics(fx: Double? = 0.0, fy: Double? = 0.0, cx: Double? = 0.0, cy: Double? = 0.0) {
    val fx: Double
    val fy: Double
    val cx: Double
    val cy: Double

    init {
        this.fx = fx ?: 0.0
        this.fy = fy ?: 0.0
        this.cx = cx ?: 0.0
        this.cy = cy ?: 0.0
    }
}