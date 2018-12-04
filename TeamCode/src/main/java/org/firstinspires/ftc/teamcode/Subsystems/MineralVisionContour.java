package org.firstinspires.ftc.teamcode.Subsystems;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class MineralVisionContour extends OpenCVPipeline {
    private int imageWidth = 480;
    private int imageHeight = 854;
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
        imageHeight = rgba.height();
        imageWidth = rgba.width();
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
                new Point(3, 3));

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

    public boolean getGoldPos() {
        try {
            MatOfPoint2f temp = new MatOfPoint2f();
            for (int i = 1; i < contours.size(); i++) {
                MatOfPoint contour = contours.get(i);
                MatOfInt hull = new MatOfInt();
                Imgproc.convexHull(contour, hull);
                MatOfPoint hullCont = new MatOfPoint();
                ArrayList<Point> hullPts = new ArrayList<>();
                for (int j : hull.toArray()) {
                    hullPts.add(new Point(contour.get(0, j)));
                }
                hullCont.fromList(hullPts);
                double area = Imgproc.contourArea(hullCont);
                if(area > 100 && area < 1000) { //placeholder areas
                    contour.convertTo(temp, CvType.CV_32F);
                    Rect bounding = Imgproc.boundingRect(hullCont);
                    if(area > 0.7 * bounding.area() && area < 1 * bounding.area()) {
                        if(Math.abs(bounding.y + bounding.height / 2 - imageHeight) < 20)
                        return true;
                    }
                }
            }
            return false;
        } catch (NullPointerException e) {
            return false;
        }
    }
}
