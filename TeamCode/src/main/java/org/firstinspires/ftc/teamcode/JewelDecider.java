package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.content.Context;
import android.util.Log;
import android.view.SurfaceView;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import static org.opencv.core.CvType.CV_8UC1;

/**
 * Created by Travis on 11/30/2017.
 */

public class JewelDecider implements CameraBridgeViewBase.CvCameraViewListener2 {

    Telemetry telementry;
    CameraBridgeViewBase mOpenCvCameraView;
    static final int DOWNSAMPLE = 2;
    private int blueRight = 0;
    volatile int redRight = 0;
    volatile int totalFrames = 0;
    private boolean started = false;

    static class RED {
        static final int HUE_LO_1 = 0;
        static final int HUE_LO_2 = 170; // 340
        static final int SAT_LO = 180;     // 60 % // original 70
        static final int VAL_LO = 100;   // 70% // oroignal 50
        static final int HUE_HI_1 = 4;
        static final int HUE_HI_2 = 180;
        static final int SAT_HI = 255;    // 100 %
        static final int VAL_HI = 255; // 100%
    }

    static class BLUE {
        static final int HUE_LO = 100;
        static final int SAT_LO = 70;    // 70 %
        static final int VAL_LO = 50;  // 70%
        static final int HUE_HI = 130;
        static final int SAT_HI = 255;  // 100 %
        static final int VAL_HI = 255; // 100%
    }

    JewelDecider(Telemetry tele, Context contxt)
    {
        telementry = tele;
        Activity act = (Activity) contxt;
        mOpenCvCameraView = (CameraBridgeViewBase) act.findViewById(R.id.opencvCameraView);
    }

    public void startCapture()
    {
        if (!started) {
         //   mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
            mOpenCvCameraView.setCvCameraViewListener(this);
            mOpenCvCameraView.enableView();
            started = true;
        }
    }

    public void stopCapture()
    {
        if (started) {
            mOpenCvCameraView.disableView();
            started = false;
        }
    }

    public boolean RedJewelRight()
    {
        double redRightProb = (double) redRight / (double) totalFrames;
        double blueRightProb = 1.0 - redRightProb;
        telementry.addData("JewelFinder", "RedRight: " + redRightProb + " | BlueRight: " + blueRightProb);
        return redRightProb > blueRightProb;
    }

    @Override
    public void onCameraViewStarted(int width, int height) {
        telementry.addData("Opencv", "Camera View started with %d x %d", width, height);
        telementry.update();
        Log.i("OPENCV", "Started CAMERA VIEW");
    }

    @Override
    public void onCameraViewStopped() {
    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        Log.i("OPENCV", "GOT FRAME");

        Mat inframe = inputFrame.rgba();
        Mat outframe = inputFrame.rgba().clone();

        // preprocess image
        Imgproc.resize(inframe, inframe, new Size(inframe.cols() / DOWNSAMPLE, inframe.rows() / DOWNSAMPLE));
        Imgproc.cvtColor(inframe, inframe, Imgproc.COLOR_RGB2HSV);
        Imgproc.blur(inframe, inframe, new Size(10, 10));

        // create red / blue masks
        Mat mask1 = new Mat(inframe.rows(), inframe.cols(), CV_8UC1);
        Mat mask2 = new Mat(inframe.rows(), inframe.cols(), CV_8UC1);

        Core.inRange(
                inframe,
                new Scalar(RED.HUE_LO_1, RED.SAT_LO, RED.VAL_LO),
                new Scalar(RED.HUE_HI_1, RED.SAT_HI, RED.VAL_HI),
                mask1
                );
        Core.inRange(
                inframe,
                new Scalar(RED.HUE_LO_2, RED.SAT_LO, RED.VAL_LO),
                new Scalar(RED.HUE_HI_2, RED.SAT_HI, RED.VAL_HI),
                mask2);

        Core.bitwise_or(mask1, mask2, mask1); // final mask red = mask1

        Core.inRange(
                inframe,
                new Scalar(BLUE.HUE_LO, BLUE.SAT_LO, BLUE.VAL_LO),
                new Scalar(BLUE.HUE_HI, BLUE.SAT_HI, BLUE.VAL_HI),
                mask2
        );

        // dilate erode
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new  Size(5, 5));
        Imgproc.erode(mask1, mask1, kernel);
        Imgproc.erode(mask2, mask2, kernel);
        Imgproc.dilate(mask1, mask1, kernel);
        Imgproc.dilate(mask2, mask2, kernel);

        // find contours
        List<MatOfPoint> redContours = new ArrayList<MatOfPoint>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask1, redContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        List<MatOfPoint> blueContours = new ArrayList<MatOfPoint>();
        Mat hierarchy2 = new Mat();
        Imgproc.findContours(mask2, blueContours, hierarchy2, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        Rect redRect = new Rect(0, 0, 0, 0);
        Rect blueRect = new Rect(0, 0, 0, 0);
        MatOfPoint2f  approxCurve = new MatOfPoint2f();
        //For each contour found
        for (int i = 0; i < redContours.size(); i++)
        {
            //Convert contours(i) from MatOfPoint to MatOfPoint2f
            MatOfPoint2f contour2f = new MatOfPoint2f(redContours.get(i).toArray());

            //Processing on mMOP2f1 which is in type MatOfPoint2f
            double approxDistance = Imgproc.arcLength(contour2f, true)*0.02;
            Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);

            //Convert back to MatOfPoint
            MatOfPoint points = new MatOfPoint( approxCurve.toArray() );

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(points);
            if (rect.area() > redRect.area())
                redRect = rect;
        }

        for (int i = 0; i < blueContours.size(); i++)
        {
            //Convert contours(i) from MatOfPoint to MatOfPoint2f
            MatOfPoint2f contour2f = new MatOfPoint2f(blueContours.get(i).toArray());

            //Processing on mMOP2f1 which is in type MatOfPoint2f
            double approxDistance = Imgproc.arcLength(contour2f, true)*0.02;
            Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);

            //Convert back to MatOfPoint
            MatOfPoint points = new MatOfPoint( approxCurve.toArray() );

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(points);
            if (rect.area() > blueRect.area())
                blueRect = rect;
        }

        // draw largest red rect
        Imgproc.rectangle(
                outframe,
                new Point(redRect.x * DOWNSAMPLE, redRect.y * DOWNSAMPLE),
                new Point((redRect.x + redRect.width) * DOWNSAMPLE, (redRect.y + redRect.height) * DOWNSAMPLE),
                new Scalar(255, 0, 0),
                5
        );

        // draw larget blue rect
        Imgproc.rectangle(
                outframe,
                new Point(blueRect.x * DOWNSAMPLE, blueRect.y * DOWNSAMPLE),
                new Point((blueRect.x + blueRect.width) * DOWNSAMPLE, (blueRect.y + blueRect.height) * DOWNSAMPLE),
                new Scalar(0, 0, 255),
                5
        );

        if (redRect.y + redRect.height / 2  < blueRect.y + blueRect.height / 2)
            redRight++;
        else
            blueRight++;

        totalFrames += 1;

        //return null;*/
        //Imgproc.resize(inframe, inframe, new Size(inframe.cols() * DOWNSAMPLE, inframe.rows() * DOWNSAMPLE));

        return outframe ;//inputFrame.rgba();
    }
}
