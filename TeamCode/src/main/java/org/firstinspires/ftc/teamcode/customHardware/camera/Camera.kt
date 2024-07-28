package org.firstinspires.ftc.teamcode.customHardware.camera

import android.util.Size

class Camera(name: String, size: Size, lensIntrinsics: LensIntrinsics) {
    var name: String
    var size: Size
    var lensIntrinsics: LensIntrinsics

    init {
        this.name = name
        this.size = size
        this.lensIntrinsics = lensIntrinsics
    }
}
