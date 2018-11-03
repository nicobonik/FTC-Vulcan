package org.firstinspires.ftc.teamcode.Subsystems;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class MineralVisionHough extends OpenCVPipeline {
    private int imageWidth, imageHeight;
    private boolean showContours = false;
    Mat main;
    Mat circles = new Mat();
    Telemetry telem;

    public synchronized void setShowCountours(boolean enabled) {
        showContours = enabled;
    }

    public void setTelem(Telemetry t) {
        telem = t;
    }

    public synchronized Mat getCircles() {
        return circles;
    }

    public Mat processFrame(Mat rgba, Mat gray) {
        imageWidth = rgba.width();
        imageHeight = rgba.height();
        Imgproc.resize(rgba, rgba, new Size(imageWidth, imageHeight * 4 / 3));
        Imgproc.resize(gray, gray, new Size(imageWidth, imageHeight * 4 / 3));
        circles = new Mat();
        Imgproc.HoughCircles(gray, circles, Imgproc.CV_HOUGH_GRADIENT, 2, 200, 100, 100, 20, 50);

        double[] circle1 = circles.get(0,0);
        double[] circle2 = circles.get(0,1);
        telem.addData("exception", "none");
        main = rgba;
        try {
            if(showContours) {
                Imgproc.circle(rgba, new Point(circle1[0], circle1[1]), (int) circle1[2], new Scalar(0, 0, 255), 3);
                Imgproc.circle(rgba, new Point(circle2[0], circle2[1]), (int) circle2[2], new Scalar(255, 0, 0), 3);
            }
        } catch (NullPointerException e) {
            telem.addData("exception", "two circles not found");
            telem.update();
            return rgba;
        }

        return rgba;
    }

    public int getGoldPos() {
        try {
            double x1, x2, y1, y2;
            if(circles.get(0, 0)[0] < circles.get(0, 1)[0]) {
                x1 = circles.get(0, 0)[0];
                x2 = circles.get(0, 1)[0];
                y1 = circles.get(0, 0)[1];
                y2 = circles.get(0, 1)[1];
            } else {
                x2 = circles.get(0, 0)[0];
                x1 = circles.get(0, 1)[0];
                y2 = circles.get(0, 0)[1];
                y1 = circles.get(0, 1)[1];
            }

            int xleft = (int)(x1 - (x2 - x1));
            int xmid = (int)((x1 + x2) / 2);
            int xright = (int)(x2 + (x2 - x1));
            int yleft = (int)(y1 - (y2 - y1));
            int ymid = (int)((y1 + y2) / 2);
            int yright = (int)(y2 + (y2 - y1));
            if(localColorGold(main, xleft, yleft, 5)) {
                return 0;
            } else if(localColorGold(main, xmid, ymid, 5)) {
                return 1;
            } else if(localColorGold(main, xright, yright, 5)) {
                return 2;
            }
            return 3;
        } catch (NullPointerException e) {
            return 3;
        }
    }

    public int getGoldPosBackup(Mat rgba) {
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

    private boolean localColorGold(Mat rgba, int x, int y, int radius) {
        Mat localRegion = rgba.submat(new Rect(x - radius, y - radius, 2 * radius, 2 * radius));
        Imgproc.cvtColor(localRegion, localRegion, Imgproc.COLOR_RGB2HLS);
        Scalar mean = Core.mean(localRegion);
        if(mean.val[0] > 40 && mean.val[0] < 70 &&
            mean.val[1] > 75 && mean.val[1] < 200 &&
            mean.val[2] > 150 && mean.val[2] < 255) {
            return true;
        }
        return false;
    }
}
