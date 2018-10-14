package org.firstinspires.ftc.teamcode.Subsystems;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class MineralVisionContour extends OpenCVPipeline {
    private final int imageWidth = 1080;
    private boolean showContours = false;
    // To keep it such that we don't have to instantiate a new Mat every call to processFrame,
    // we declare the Mats up here and reuse them. This is easier on the garbage collector.
    private Mat hsv = new Mat();
    private Mat thresholded = new Mat();

    // this is just here so we can expose it later thru getContours.
    private List<MatOfPoint> contours = new ArrayList<>();

    public synchronized void setShowCountours(boolean enabled) {
        showContours = enabled;
    }

    public synchronized List<MatOfPoint> getContours() {
        return contours;
    }

    // This is called every camera frame.
    @Override
    public Mat processFrame(Mat rgba, Mat gray) {
        // First, we change the colorspace from RGBA to HSV, which is usually better for color
        Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV, 3);
        // Then, we threshold our hsv image so that we get a black/white binary image where white
        // is the blues listed in the specified range of values
        // you can use a program like WPILib GRIP to find these values, or just play around.
        Core.inRange(hsv, new Scalar(0, 0, 180), new Scalar(0, 80, 255), thresholded);

        // we blur the thresholded image to remove noise
        // there are other types of blur like box blur or gaussian which can be explored.
        //Imgproc.blur(thresholded, thresholded, new Size(3, 3));

        Mat element = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
                new Size(7, 7),
                new Point(3d, 3d));

        Imgproc.erode(thresholded, thresholded, element);
        Imgproc.dilate(thresholded, thresholded, element);
        // create a list to hold our contours.
        // Conceptually, there is going to be a single contour for the outline of every blue object
        // that we can find. We can iterate over them to find objects of interest.
        // the Imgproc module has many functions to analyze individual contours by their area, avg position, etc.
        contours = new ArrayList<>();
        // this function fills our contours variable with the outlines of blue objects we found
        Imgproc.findContours(thresholded, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        // Then we display our nice little binary threshold on screen
        if (showContours) {
            // this draws the outlines of the blue contours over our original image.
            // they are highlighted in green.
            Imgproc.drawContours(rgba, contours, -1, new Scalar(0, 255, 0), 2, 8);
        }

        return rgba;
    }

    public int getGoldPos() {
        try {
            double max1 = 0;
            double max2 = 0;
            double x1 = 0;
            double x2 = 0;
            double area;
            float[] radius = {};
            double circleArea;
            MatOfPoint2f temp = new MatOfPoint2f();
            Point center = new Point();
            for (MatOfPoint contour : contours) {
                area = Imgproc.contourArea(contour);
                if (area > 500 && area < 1000) { //placeholder areas
                    contour.convertTo(temp, CvType.CV_32F);
                    Imgproc.minEnclosingCircle(temp, center, radius);
                    circleArea = Math.PI * radius[0] * radius[0];
                    if (area > 0.9 * circleArea * circleArea && area < 1.15 * circleArea * circleArea) {
                        if (area > max1) {
                            max2 = max1;
                            x2 = x1;
                            max1 = area;
                            x1 = center.x;
                        } else if (area > max2) {
                            max2 = area;
                            x2 = center.x;
                        }
                    }
                }
            }
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
