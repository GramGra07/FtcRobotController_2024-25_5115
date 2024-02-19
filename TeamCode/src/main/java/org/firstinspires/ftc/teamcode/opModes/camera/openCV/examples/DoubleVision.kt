/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.opModes.camera.openCV.examples

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.EOCVWebcam
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.firstinspires.ftc.vision.tfod.TfodProcessor

/*
 * This OpMode illustrates the basics of using both AprilTag recognition and TensorFlow
 * Object Detection.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "Concept: Double Vision", group = "Concept")
@Disabled
class DoubleVision : LinearOpMode() {
    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private var aprilTag: AprilTagProcessor? = null

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private var tfod: TfodProcessor? = null

    /**
     * The variable to store our instance of the vision portal.
     */
    private var myVisionPortal: VisionPortal? = null
    override fun runOpMode() {
        initDoubleVision()

        // This OpMode loops continuously, allowing the user to switch between
        // AprilTag and TensorFlow Object Detection (TFOD) image processors.
        while (!isStopRequested) {
            if (opModeInInit()) {
                telemetry.addData("DS preview on/off", "3 dots, Camera Stream")
                telemetry.addLine()
                telemetry.addLine("----------------------------------------")
            }
            if (myVisionPortal!!.getProcessorEnabled(aprilTag)) {
                // User instructions: Dpad left or Dpad right.
                telemetry.addLine("Dpad Left to disable AprilTag")
                telemetry.addLine()
                telemetryAprilTag()
            } else {
                telemetry.addLine("Dpad Right to enable AprilTag")
            }
            telemetry.addLine()
            telemetry.addLine("----------------------------------------")
            if (myVisionPortal!!.getProcessorEnabled(tfod)) {
                telemetry.addLine("Dpad Down to disable TFOD")
                telemetry.addLine()
                telemetryTfod()
            } else {
                telemetry.addLine("Dpad Up to enable TFOD")
            }

            // Push telemetry to the Driver Station.
            telemetry.update()
            if (gamepad1.dpad_left) {
                myVisionPortal!!.setProcessorEnabled(aprilTag, false)
            } else if (gamepad1.dpad_right) {
                myVisionPortal!!.setProcessorEnabled(aprilTag, true)
            }
            if (gamepad1.dpad_down) {
                myVisionPortal!!.setProcessorEnabled(tfod, false)
            } else if (gamepad1.dpad_up) {
                myVisionPortal!!.setProcessorEnabled(tfod, true)
            }
            sleep(20)
        } // end while loop
    } // end method runOpMode()

    /**
     * Initialize AprilTag and TFOD.
     */
    private fun initDoubleVision() {
        // -----------------------------------------------------------------------------------------
        // AprilTag Configuration
        // -----------------------------------------------------------------------------------------
        aprilTag = AprilTagProcessor.Builder()
            .build()

        // -----------------------------------------------------------------------------------------
        // TFOD Configuration
        // -----------------------------------------------------------------------------------------
        tfod = TfodProcessor.Builder()
            .build()

        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------------------------------------------------------------------------------------
        myVisionPortal = if (USE_WEBCAM) {
            VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName::class.java, EOCVWebcam.cam1_N))
                .addProcessors(tfod, aprilTag)
                .build()
        } else {
            VisionPortal.Builder()
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessors(tfod, aprilTag)
                .build()
        }
    } // end initDoubleVision()

    /**
     * Add telemetry about AprilTag detections.
     */
    private fun telemetryAprilTag() {
        val currentDetections: List<AprilTagDetection> = aprilTag!!.detections
        telemetry.addData("# AprilTags Detected", currentDetections.size)

        // Step through the list of detections and display info for each one.
        for (detection in currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(
                    String.format(
                        "\n==== (ID %d) %s",
                        detection.id,
                        detection.metadata.name
                    )
                )
                telemetry.addLine(
                    String.format(
                        "XYZ %6.1f %6.1f %6.1f  (inch)",
                        detection.ftcPose.x,
                        detection.ftcPose.y,
                        detection.ftcPose.z
                    )
                )
                telemetry.addLine(
                    String.format(
                        "PRY %6.1f %6.1f %6.1f  (deg)",
                        detection.ftcPose.pitch,
                        detection.ftcPose.roll,
                        detection.ftcPose.yaw
                    )
                )
                telemetry.addLine(
                    String.format(
                        "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)",
                        detection.ftcPose.range,
                        detection.ftcPose.bearing,
                        detection.ftcPose.elevation
                    )
                )
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id))
                telemetry.addLine(
                    String.format(
                        "Center %6.0f %6.0f   (pixels)",
                        detection.center.x,
                        detection.center.y
                    )
                )
            }
        } // end for() loop
    } // end method telemetryAprilTag()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private fun telemetryTfod() {
        val currentRecognitions = tfod!!.recognitions
        telemetry.addData("# Objects Detected", currentRecognitions.size)

        // Step through the list of recognitions and display info for each one.
        for (recognition in currentRecognitions) {
            val x = ((recognition.left + recognition.right) / 2).toDouble()
            val y = ((recognition.top + recognition.bottom) / 2).toDouble()
            telemetry.addData("", " ")
            telemetry.addData(
                "Image",
                "%s (%.0f %% Conf.)",
                recognition.label,
                recognition.confidence * 100
            )
            telemetry.addData("- Position", "%.0f / %.0f", x, y)
            telemetry.addData("- Size", "%.0f x %.0f", recognition.width, recognition.height)
        } // end for() loop
    } // end method telemetryTfod()

    companion object {
        private const val USE_WEBCAM = true // true for webcam, false for phone camera
    }
} // end class
