package org.firstinspires.ftc.teamcode.Subsystems;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;

public class MineralVisionHough extends OpenCVPipeline {
    private final int imageWidth = 1080;
    private boolean showContours;
    Mat circles = new Mat();

    public synchronized void setShowCountours(boolean enabled) {
        showContours = enabled;
    }

    public synchronized Mat getCircles() {
        return circles;
    }

    public Mat processFrame(Mat rgba, Mat gray) {
        circles = new Mat();
        Imgproc.HoughCircles(gray, circles, Imgproc.CV_HOUGH_GRADIENT, 2, 200, 100, 100, 300, 400);

        Imgproc.circle(rgba, new Point(circles.get(0,0)[0], circles.get(0,0)[1]), (int)circles.get(0,0)[2], new Scalar(0, 0, 255));
        Imgproc.circle(rgba, new Point(circles.get(0,1)[0], circles.get(0,1)[1]), (int)circles.get(0,1)[2], new Scalar(255, 0, 0));

        return rgba;
    }

    public int getGoldPos() {
        try {
            double x1 = circles.get(0, 0)[0];
            double x2 = circles.get(0, 1)[0];
            if ((x2 + x1) / 2 < imageWidth / 3) {
                return 2;
            } else if ((x2 + x1) / 2 < imageWidth * 2 / 3) {
                return 1;
            } else {
                return 0;
            }
        } catch (NullPointerException e) {
            return 3;
        }
    }
}
