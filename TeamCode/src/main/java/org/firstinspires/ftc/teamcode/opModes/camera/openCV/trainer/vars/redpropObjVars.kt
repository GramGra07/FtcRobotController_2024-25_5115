package org.firstinspires.ftc.teamcode.opModes.camera.openCV.trainer.vars

//@Config
object redpropObjVars {
    var aspectRatio = 1.0
    var minWidth = 109.0
    var minHeight = 74.0
    var maxWidth = 155.0
    var maxHeight = 153.0
    var minArea = minWidth * minHeight
    var maxArea = maxWidth * maxHeight
    var tolerance =
        0.2 // this is the value that will determine how far off the aspect ratio can be to still detect it, you will need to tune it more
    var translationX = 0.06896551724137931
    var translationY = 0.15
}