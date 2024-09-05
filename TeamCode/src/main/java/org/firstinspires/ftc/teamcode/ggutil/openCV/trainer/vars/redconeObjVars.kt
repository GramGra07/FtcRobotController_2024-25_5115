package org.firstinspires.ftc.teamcode.ggutil.openCV.trainer.vars

//@Config
object redconeObjVars {
    var aspectRatio = 0.7872340425531915
    var minWidth = 43.0
    var minHeight = 58.0
    var maxWidth = 108.0
    var maxHeight = 145.0
    var minArea = minWidth * minHeight
    var maxArea = maxWidth * maxHeight
    var tolerance =
        0.2 // this is the value that will determine how far off the aspect ratio can be to still detect it, you will need to tune it more
    var translationX = 0.05357142857142857
    var translationY = 0.5
}