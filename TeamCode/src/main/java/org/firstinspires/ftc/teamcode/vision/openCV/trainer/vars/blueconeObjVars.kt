package org.firstinspires.ftc.teamcode.vision.openCV.trainer.vars

//@Config
object blueconeObjVars {
    var aspectRatio = 0.7333333333333333
    var minWidth = 53.0
    var minHeight = 74.0
    var maxWidth = 228.0
    var maxHeight = 240.0
    var minArea = minWidth * minHeight
    var maxArea = maxWidth * maxHeight
    var tolerance =
        0.2 // this is the value that will determine how far off the aspect ratio can be to still detect it, you will need to tune it more
    var translationX = 1.7142857142857142
    var translationY = 8.033898305084746
}