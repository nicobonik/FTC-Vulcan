package org.firstinspires.ftc.teamcode.Subsystems;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvException;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class MineralVisionHough extends OpenCVPipeline {
    private int imageWidth, imageHeight;
    private boolean showContours = false;
    private Mat main;
    private Mat circles = new Mat();
    private Telemetry telem;
    private double[] circle1, circle2;
    private int[][] candidatePos = new int[3][2];
    private int cannyThresh = 120;
    private int houghThresh = 180;

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
        Core.inRange(gray, new Scalar(120), new Scalar(200), gray);
        circles = new Mat();
        Imgproc.HoughCircles(gray, circles, Imgproc.CV_HOUGH_GRADIENT, 2, 50, cannyThresh, houghThresh, 30, 100);
        if(circles.cols() < 2) {
            cannyThresh--;
            houghThresh--;
            return rgba;
        } else if(circles.cols() > 2) {
            cannyThresh++;
            houghThresh++;
        }
        circle1 = circles.get(0,0);
        circle2 = circles.get(0,1);
        main = rgba;
        //
        double x1, x2, y1, y2;
        if(circle1[1] < circle2[1]) {
            x1 = circle1[0];
            x2 = circle2[0];
            y1 = circle1[1];
            y2 = circle2[1];
        } else {
            x2 = circle1[0];
            x1 = circle2[0];
            y2 = circle1[1];
            y1 = circle2[1];
        }

        candidatePos[0][0] = (int)(x1 - (x2 - x1));
        candidatePos[1][0] = (int)((x1 + x2) / 2);
        candidatePos[2][0] = (int)(x2 + (x2 - x1));
        candidatePos[0][1] = (int)(y1 - (y2 - y1));
        candidatePos[1][1] = (int)((y1 + y2) / 2);
        candidatePos[2][1] = (int)(y2 + (y2 - y1));
        //
        if(showContours) {
            Imgproc.circle(rgba, new Point(circle1[0], circle1[1]), (int) circle1[2], new Scalar(0, 0, 255), 3);
            Imgproc.circle(rgba, new Point(circle2[0], circle2[1]), (int) circle2[2], new Scalar(255, 0, 0), 3);
            int position = getGoldPos();
            if(position == -1) {
                Imgproc.circle(rgba, new Point(candidatePos[0][0], candidatePos[0][1]), 10, new Scalar(0, 255, 0), 2);
                Imgproc.circle(rgba, new Point(candidatePos[1][0], candidatePos[1][1]), 10, new Scalar(0, 255, 0), 2);
                Imgproc.circle(rgba, new Point(candidatePos[2][0], candidatePos[2][1]), 10, new Scalar(0, 255, 0), 2);
            } else {
                Imgproc.circle(rgba, new Point(candidatePos[position][0], candidatePos[position][1]), 10, new Scalar(0, 255, 0), 2);
            }
        }
        return rgba;
    }

    public int getGoldPos() {
        try {
            if(localColorGold(main, candidatePos[0][0], candidatePos[0][1], 5)) {
                return 0;
            } else if(localColorGold(main, candidatePos[1][0], candidatePos[1][1], 5)) {
                return 1;
            } else if(localColorGold(main, candidatePos[2][0], candidatePos[2][1], 5)) {
                return 2;
            }
            return -1;
        } catch (Exception e) {
            return -1;
        }
    }

    public int getGoldPosBackup(Mat rgba) {
        try {
            double x1 = circle1[0];
            double x2 = circle2[0];
            if ((x2 + x1) / 2 < imageWidth / 3) {
                return 2;
            } else if ((x2 + x1) / 2 < imageWidth * 2 / 3) {
                return 1;
            } else {
                return 0;
            }
        } catch (Exception e) {
            return -1;
        }
    }

    private boolean localColorGold(Mat rgba, int x, int y, int squareRadius) {
        try {
            Mat localRegion = rgba.submat(new Rect(x - squareRadius, y - squareRadius, 2 * squareRadius, 2 * squareRadius));
            Imgproc.cvtColor(localRegion, localRegion, Imgproc.COLOR_RGB2HLS);
            Scalar mean = Core.mean(localRegion);
            if (mean.val[0] > 10 && mean.val[0] < 30 &&
                    mean.val[1] > 90 && mean.val[1] < 160 &&
                    mean.val[2] > 115 && mean.val[2] < 200) {
                return true;
            }
            return false;
        } catch (CvException e) {
            return false;
        }
    }
}
