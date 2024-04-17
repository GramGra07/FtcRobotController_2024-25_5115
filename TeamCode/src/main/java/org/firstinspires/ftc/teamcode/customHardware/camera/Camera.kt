package org.firstinspires.ftc.teamcode.customHardware.camera

import android.util.Size

class Camera(size: Size, lensIntrinsics: LensIntrinsics) {
    var size: Size
    var lensIntrinsics: LensIntrinsics

    init {
        this.size = size
        this.lensIntrinsics = lensIntrinsics
    }
}
