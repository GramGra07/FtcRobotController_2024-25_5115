/*
 * Copyright (c) 2023 FIRST
 *
 * All rights reserved.
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
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.ggutil

import android.util.Size
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.EOCVWebcam
import org.firstinspires.ftc.vision.VisionPortal
import java.util.Locale

@TeleOp(name = "Utility: Camera Frame Capture", group = "aaa")
@Disabled
class UtilityCameraFrameCapture : LinearOpMode() {
    val RESOLUTION_WIDTH = 1280
    val RESOLUTION_HEIGHT = 720

    // Internal state
    var lastX = false
    var frameCount = 0
    var capReqTime: Long = 0
    override fun runOpMode() {
        val portal: VisionPortal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, EOCVWebcam.cam2_N))
            .setCameraResolution(Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
            .build()
        while (!isStopRequested) {
            val x = gamepad1.cross
            if (x && !lastX) {
                portal.saveNextFrameRaw(
                    String.format(
                        Locale.US,
                        "CameraFrameCapture-%06d",
                        frameCount++
                    )
                )
                capReqTime = System.currentTimeMillis()
            }
            lastX = x
            telemetry.addLine("######## Camera Capture Utility ########")
            telemetry.addLine(
                String.format(
                    Locale.US,
                    " > Resolution: %dx%d",
                    RESOLUTION_WIDTH,
                    RESOLUTION_HEIGHT
                )
            )
            telemetry.addLine(" > Press X (or Square) to capture a frame")
            telemetry.addData(" > Camera Status", portal.cameraState)
            if (capReqTime != 0L) {
                telemetry.addLine("\nCaptured Frame!")
            }
            if (capReqTime != 0L && System.currentTimeMillis() - capReqTime > 1000) {
                capReqTime = 0
            }
            telemetry.update()
        }
    }
}
