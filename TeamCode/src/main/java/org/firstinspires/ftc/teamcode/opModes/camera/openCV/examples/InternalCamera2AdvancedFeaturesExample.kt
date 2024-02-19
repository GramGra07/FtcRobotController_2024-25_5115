/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
package org.firstinspires.ftc.teamcode.opModes.camera.openCV.examples

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvInternalCamera2
import org.openftc.easyopencv.OpenCvPipeline

/**
 * In this sample, we demonstrate how to use the advanced features provided
 * by the [OpenCvInternalCamera2] interface
 */
@TeleOp
@Disabled
class InternalCamera2AdvancedFeaturesExample : LinearOpMode() {
    /**
     * NB: we declare our camera as the [OpenCvInternalCamera2] type,
     * as opposed to simply [OpenCvCamera]. This allows us to access
     * the advanced features supported only by the internal camera.
     */
    lateinit var phoneCam: OpenCvInternalCamera2
    override fun runOpMode() {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCV,
         * you should take a look at [InternalCamera1Example] or its
         * webcam counterpart, [OpenCVBlue] first.
         */
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName
        )
        phoneCam = OpenCvCameraFactory.getInstance()
            .createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId)
        phoneCam.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                phoneCam.setPipeline(UselessColorBoxDrawingPipeline(Scalar(255.0, 0.0, 0.0)))

                /*
                 * Start streaming
                 */phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)

                /*
                 * Demonstrate how to turn on the flashlight
                 */phoneCam.setFlashlightEnabled(true)

                /*
                 * Demonstrate how to lock the camera hardware to sending frames at 30FPS
                 */phoneCam.setSensorFps(30)

                /*
                 * Demonstrate how to set some manual sensor controls
                 */phoneCam.setExposureMode(OpenCvInternalCamera2.ExposureMode.MANUAL)
                phoneCam.setFocusMode(OpenCvInternalCamera2.FocusMode.MANUAL)
                phoneCam.setFocusDistance(phoneCam.minFocusDistance)
                phoneCam.setExposureFractional(60)
                phoneCam.setSensorGain(400)
                phoneCam.setWhiteBalanceMode(OpenCvInternalCamera2.WhiteBalanceMode.INCANDESCENT)
            }

            override fun onError(errorCode: Int) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        })
        waitForStart()
        while (opModeIsActive()) {
            sleep(100)
        }
    }

    internal inner class UselessColorBoxDrawingPipeline(var color: Scalar) : OpenCvPipeline() {
        override fun processFrame(input: Mat): Mat {
            Imgproc.rectangle(
                input,
                Point(
                    (
                            input.cols() / 4).toDouble(),
                    (
                            input.rows() / 4).toDouble()
                ),
                Point(
                    (
                            input.cols() * (3f / 4f)).toDouble(),
                    (
                            input.rows() * (3f / 4f)).toDouble()
                ),
                color, 4
            )
            return input
        }
    }
}