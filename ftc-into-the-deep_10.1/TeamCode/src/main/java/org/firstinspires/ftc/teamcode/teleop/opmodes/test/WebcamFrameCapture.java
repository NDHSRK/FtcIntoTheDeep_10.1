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

//##PY 08/31/2023 This test TeleOp OpMode is derived from the FTC 8.2 sample
// UtilityCameraFrameCapture, which writes webcam frames to a file.

package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.ftcdevcommon.platform.android.TimeStamp;
import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.device.camera.RawFrameProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

import java.util.Date;

@TeleOp(name = "Webcam Frame Capture", group = "Test")
//@Disabled
public class WebcamFrameCapture extends LinearOpMode {

    //## Hardcoded here but in the real world the resolution will
    // come from RobotConfig.xml. In order for this to happen this
    // class will have to implement TeleOpWithAlliance, which gives
    // access to FTCRobot where the camera configurations are stored.
    private final int RESOLUTION_WIDTH = 640;
    private final int RESOLUTION_HEIGHT = 480;

    @Override
    public void runOpMode() throws InterruptedException {
        //## Image directory hardcoded; should be RobotConstants.imageDir
        String imageWorkingDirectory = WorkingDirectory.getWorkingDirectory() + "/images/";
        RawFrameProcessor rawFrameProcessor = new RawFrameProcessor();
        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(false)
                // If set "false", monitor shows camera view without annotations.
                .setAutoStopLiveView(false)
                // Set and enable the processor.
                .addProcessor(rawFrameProcessor)
                .build();

        // Make sure that frames are flowing within 3 seconds.
        boolean framesFlowing = false;
        for (int i = 0; i < 3; i++) {
            framesFlowing = rawFrameProcessor.rawFramesFlowing();
            if (framesFlowing) {
                telemetry.addLine("######## Webcam Frame Capture ########");
                telemetry.addLine(" > Press X to capture a frame");
                telemetry.addData(" > Camera Status", portal.getCameraState());
                telemetry.update();
                break;
            } else {
                telemetry.addLine("No frames for 1 sec; retry");
                telemetry.addLine("Camera status " + portal.getCameraState());
                telemetry.update();
            }
        }

        if (!framesFlowing) {
            telemetry.addLine("No frames flowing from the camera after 3 sec");
            telemetry.update();
            return;
        }

        boolean x;
        boolean lastX = false;
        int frameCaptureNumber = 1;
        Pair<Mat, Date> frameToWrite;
        ElapsedTime dataAcquiredTimer = new ElapsedTime();
        while (!isStopRequested()) {
            x = gamepad1.x;
            if (x && !lastX) {
                // Get a frame from the webcam frame processor.
                frameToWrite = null;
                dataAcquiredTimer.reset(); // start
                while (dataAcquiredTimer.milliseconds() < 2000) {
                    frameToWrite = rawFrameProcessor.getImage();
                    if (frameToWrite != null)
                        break;
                    else {
                        telemetry.addLine("No available webcam frame within 1 sec");
                        telemetry.update();
                        sleep(50);
                    }
                }

                if (frameToWrite == null) {
                    telemetry.addLine("Two-second timer expired while waiting for a frame!");
                    telemetry.update();
                }
                else {
                    telemetry.addLine("Captured Frame " + frameCaptureNumber++);
                    telemetry.update();

                    //## As a test write it to FIRST\TeamData\images
                    String fileDate = TimeStamp.getDateTimeStamp(frameToWrite.second);
                    Imgcodecs.imwrite(imageWorkingDirectory + "WebcamFrame" + "_" + fileDate + ".png", frameToWrite.first);
                }
            }

            lastX = x;
        }
    }
}
