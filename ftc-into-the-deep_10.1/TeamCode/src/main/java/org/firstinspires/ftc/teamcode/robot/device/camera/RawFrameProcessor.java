//##PY 08/31/2023 This class is derived from the FTC 8.2 April Tag
// implementation AprilTagProcessor.java.

package org.firstinspires.ftc.teamcode.robot.device.camera;

import android.graphics.Canvas;

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.Date;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.ReentrantLock;

public class RawFrameProcessor implements VisionProcessor, ImageProvider {
    private final CountDownLatch framesAvailableLatch = new CountDownLatch(1);
    private final Mat bgrFrame = new Mat();
    private final ReentrantLock rawFrameLock = new ReentrantLock();
    private final Condition rawFrameAvailableCondition = rawFrameLock.newCondition();
    private boolean rawFrameAvailable = false;
    private Pair<Mat, Date> rawFrame;

    //## This is a callback. It definitely runs on another thread.
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Init is called from VisionPortalImpl when the first frame for this
        // processor has been received; the frame itself is not passed in
        // here. But we can signal that frames are flowing.
        framesAvailableLatch.countDown();
    }

    //## This is a callback; assume it's running on another thread.
    // So store the frame in an AtomicReference.
    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        // From the EasyOpenCV readme:
        // **IMPORTANT NOTE:** EasyOpenCV delivers RGBA frames
        // So we need to convert to BGR for OpenCV here.
        Imgproc.cvtColor(input, bgrFrame, Imgproc.COLOR_RGBA2BGR);

        try {
            // Post the raw frame.
            rawFrameLock.lock();
            rawFrame = Pair.create(bgrFrame, new Date());
            rawFrameAvailable = true;
            rawFrameAvailableCondition.signal();
        } finally {
            rawFrameLock.unlock();
        }

        return input;
    }

    //## This is a callback.
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        //## If you don't put in some work to draw on the Canvas then
        // in the Driver Station camera stream you will only see the
        // original image.
        // We don't draw frames for the RawFrameProcessor.
    }

    public boolean rawFramesFlowing() throws InterruptedException {
        // await returns true if the count reached zero and false if the
        // waiting time elapsed before the count reached zero.
        return framesAvailableLatch.await(1000, TimeUnit.MILLISECONDS);
    }

    // Override for the ImageProvider interface.
    @Override
    public Pair<Mat, Date> getImage() throws InterruptedException {
        try {
            rawFrameLock.lock();
            while (!rawFrameAvailable) {
                // Condition.await returns false if the waiting time detectably
                // elapsed before return from the method, else true.
                if (!rawFrameAvailableCondition.await(1000, TimeUnit.MILLISECONDS))
                    return null; // timed out
            }

            return rawFrame;

        } finally {
            rawFrame = null;
            rawFrameAvailable = false;
            rawFrameLock.unlock();
        }
    }

}

