package org.firstinspires.ftc.teamcode.Subsystems;

import android.widget.FrameLayout;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

public class Navigation {
    //Navigation
    private double x, y, hdg;

    //OpenCV
    private int imageWidth, imageHeight;
    private boolean showContours = false;
    private Mat main;
    private Mat frame;
    private byte[] frameBuffer;
    private int goldPosition;

    //Vuforia
    private static final String VUFORIA_KEY = "AaLwOCr/////AAABGav2GMoitk79tCVWogR++j4qdtG1lQgtNBy8Vvyb1hRG2re62CXytFbaSWxaTBL5d+dZlWlOL3DGc1SRxOx+FKiGaP73ct6eGoWuZiQW0RjcPCxJ1LZwPJfs5D1I+B0lUK09sGbpF5LO6yKpWG7P9TmrZ5PpB+W4Xj/hduc0c5LMX09z4acyb0GQMRXzjtdgWDvgjNp53x9nea8/UwRfeUgMN0x5Tzj9H9ALjYr54U112ev1WxcGzaC9xjpKQ96KxIDAT5h1GN8i3j0b5FVI4rp/lC17Lsjkz9thrVw0YCZXOTmdD5sZktkNIbj97B6y4BUrDkBaJNAmQiQSmG9DHVr7r9kTW3WiQ/khg1u0ciou";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
    final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
    final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    private boolean vuforiaActive = false;
    private boolean cvActive = false;

    private VuforiaLocalizer vuforia;
    private java.util.concurrent.BlockingQueue<VuforiaLocalizer.CloseableFrame>	frameQueue;

    private VuforiaTrackables targetsRoverRuckus;
    private VuforiaTrackable blueRover, redFootprint, frontCraters, backSpace;
    private VuforiaLocalizer.Parameters parameters;
    private List<VuforiaTrackable> allTrackables;

    private OpenGLMatrix blueRoverLocationOnField, redFootprintLocationOnField, frontCratersLocationOnField, backSpaceLocationOnField;
    private OpenGLMatrix phoneLocationOnRobot;

    private Runnable visionProcess;

    private Telemetry telemetry;
    /**
     * If you are standing in the Red Alliance Station looking towards the center of the field,
     *     - The X axis runs from your left to the right. (positive from the center to the right)
     *     - The Y axis runs from the Red Alliance Station towards the other side of the field
     *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
     *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
     *
     * Robot:
     * FORWARD = +X
     * LEFT = +Y
     * UP = +Z
     */
    public Navigation(int cameraMonitorViewId, Telemetry telem) {
        telemetry = telem;

        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        allTrackables = new ArrayList<>();
        allTrackables.addAll(targetsRoverRuckus);

        blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 0 : -180, 0, 0));

        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        frameQueue = vuforia.getFrameQueue();

        visionProcess = new Runnable() {
            public void run() {
                while (!Thread.currentThread().isInterrupted()) {
                    if(vuforiaActive) {
                        double[] location = getLocation();
                        x = location[0];
                    }
                    if(cvActive) {
                        goldPosition = processFrame(vuforiaToMat());
                    }
                }
            }
        };
    }

    //x, y, hdg
    private double[] getLocation() {
        // vuforia tracking
        // check all the trackable target to see which one (if any) is visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            telemetry.update();
            return new double[] {translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, rotation.thirdAngle};
        } else {
            telemetry.addData("Visible Target", "none");
            telemetry.update();
        }
        return new double[] {-1, -1, -1};
    }

    public Mat vuforiaToMat() {
        // returns null if correct image format not found or queue empty
        // grab frames and process them
        if (!frameQueue.isEmpty()) {
            VuforiaLocalizer.CloseableFrame vuforiaFrame = null;
            try {
                vuforiaFrame = frameQueue.take();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }

            if (vuforiaFrame != null) {
                for (int i = 0; i < vuforiaFrame.getNumImages(); i++) {
                    Image image = vuforiaFrame.getImage(i);
                    if (image.getFormat() == PIXEL_FORMAT.RGB888) {
                        int imageWidth = image.getWidth(), imageHeight = image.getHeight();
                        ByteBuffer byteBuffer = image.getPixels();
                        if (frameBuffer == null) {
                            frameBuffer = new byte[byteBuffer.capacity()];
                        }
                        byteBuffer.get(frameBuffer);
                        if (frame == null) {
                            frame = new Mat(imageHeight, imageWidth, CvType.CV_8UC3);
                        }
                        frame.put(0, 0, frameBuffer);

                        if (parameters.cameraDirection == VuforiaLocalizer.CameraDirection.FRONT) {
                            Core.flip(frame, frame, 1);
                        }

                        return frame;
                    }
                    telemetry.addData("correct format", "not found");
                }
                vuforiaFrame.close();
            }
        } else {
            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    public int processFrame(Mat rgb) {
        // image processing
        Mat gray = new Mat();
        Imgproc.cvtColor(frame, gray, Imgproc.COLOR_RGB2HLS);
        imageWidth = gray.width();
        imageHeight = gray.height();
        Imgproc.resize(gray, gray, new Size(imageWidth, imageHeight * 4 / 3));
        Mat circles = new Mat();
        Imgproc.HoughCircles(gray, circles, Imgproc.CV_HOUGH_GRADIENT, 2, 200, 100, 100, 20, 50);

        double[] circle1 = circles.get(0,0);
        double[] circle2 = circles.get(0,1);
        telemetry.addData("exception", "none");

        try {
            // get gold position
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
            if(localColorGold(rgb, xleft, yleft, 5)) {
                return 0;
            } else if(localColorGold(rgb, xmid, ymid, 5)) {
                return 1;
            } else if(localColorGold(rgb, xright, yright, 5)) {
                return 2;
            }
            return -1;
        } catch (NullPointerException e) {
            telemetry.addData("exception", "two circles not found");
            telemetry.update();
            return -1;
        }
    }


    /*public int getGoldPosBackup(Mat rgba) {
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
    }*/

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
