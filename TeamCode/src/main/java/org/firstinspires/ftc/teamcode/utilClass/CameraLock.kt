package org.firstinspires.ftc.teamcode.utilClass

import org.firstinspires.ftc.teamcode.extensions.VisionExtensions.cross
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.FastIntakeSubsystem
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc

class CameraLock(var color:FastIntakeSubsystem.Color, var center:Point){
    fun draw(frame: Mat){
        cross(frame,center,10,Scalar(255.0,0.0,0.0))
        Imgproc.circle(frame,center,10,Scalar(255.0,0.0,0.0),1)
    }
}