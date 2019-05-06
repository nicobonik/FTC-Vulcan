package org.firstinspires.ftc.team8375;


import android.app.Activity;
import android.graphics.Canvas;
import android.support.annotation.Nullable;
import android.view.Surface;

import java.util.ArrayList;
import java.util.List;
import java.util.Iterator;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Point;
import org.opencv.features2d.FastFeatureDetector;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.lang.reflect.Array;
import java.util.Vector;

public class OpenCVClass extends OpenCVPipeline {
    private Mat hsv = new Mat();
    private Mat threshold = new Mat();
    private Mat rgb = new Mat();
    private Mat output = new Mat();
    // private MatOfPoint output = new MatOfPoint();
    private List<MatOfPoint> contours = new ArrayList<>();
    private List<Moments> mu;

    public synchronized List<MatOfPoint> getContours() {
        return contours;
    }

    public synchronized Mat getThreshold(){
        return threshold;
    }

    public synchronized Mat getRgb() { return rgb; }

    @Override
    public Mat processFrame(Mat rgba, Mat gray) {
        //this is where image processing goes

        Imgproc.blur(rgba, hsv, new Size(60, 60));
        Imgproc.cvtColor(hsv, hsv, Imgproc.COLOR_RGB2HSV, 3);
        Core.inRange(hsv, new Scalar(16, 112, 30), new Scalar(32, 255, 255), threshold);
        Rect rectCrop = new Rect(100, 50, 200, 500);

        output = threshold.submat(rectCrop);

        findContours(output, false, contours);

        //Imgproc.cvtColor(threshold, rgba, Imgproc.COLOR_HSV2RGB, 3);
        //Imgproc.drawContours(threshold, contours, -1, new Scalar(0, 0, 255, 0), 2, 8);

// comment this rotate out when done pls
        // Core.rotate(rgba, rgba, Core.ROTATE_180);
        //rgb = rgba.submat(rectCrop);
        return rgb;
    }


    private void findContours(Mat input, boolean externalOnly,
                              List<MatOfPoint> contours) {
        Mat hierarchy = new Mat();
        contours.clear();
        int mode;
        if (externalOnly) {
            mode = Imgproc.RETR_EXTERNAL;
        }
        else {
            mode = Imgproc.RETR_LIST;
        }
        int method = Imgproc.CHAIN_APPROX_SIMPLE;
        Imgproc.findContours(input, contours, hierarchy, mode, method);
    }

}
