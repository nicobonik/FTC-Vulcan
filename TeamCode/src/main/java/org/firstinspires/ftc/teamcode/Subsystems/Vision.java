package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Bitmap;
import android.widget.FrameLayout;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvException;
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

public class Vision {
    // Location
    private volatile double x, y, hdg;
    private ElapsedTime timer;

    // OpenCV
    private MineralVisionHough mineralVision = new MineralVisionHough();
    private int goldPosition;

    // Vuforia
    private static final String VUFORIA_KEY = "AaLwOCr/////AAABGav2GMoitk79tCVWogR++j4qdtG1lQgtNBy8Vvyb1hRG2re62CXytFbaSWxaTBL5d+dZlWlOL3DGc1SRxOx+FKiGaP73ct6eGoWuZiQW0RjcPCxJ1LZwPJfs5D1I+B0lUK09sGbpF5LO6yKpWG7P9TmrZ5PpB+W4Xj/hduc0c5LMX09z4acyb0GQMRXzjtdgWDvgjNp53x9nea8/UwRfeUgMN0x5Tzj9H9ALjYr54U112ev1WxcGzaC9xjpKQ96KxIDAT5h1GN8i3j0b5FVI4rp/lC17Lsjkz9thrVw0YCZXOTmdD5sZktkNIbj97B6y4BUrDkBaJNAmQiQSmG9DHVr7r9kTW3WiQ/khg1u0ciou";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    private final int CAMERA_FORWARD_DISPLACEMENT = 110;   // eg: Camera is 110 mm in front of robot center
    private final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
    private final int CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    private boolean vuforiaActive = false;
    private boolean cvActive = false;

    private VuforiaLocalizer vuforia;
    private java.util.concurrent.BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue;

    private VuforiaTrackables targetsRoverRuckus;
    private VuforiaTrackable blueRover, redFootprint, frontCraters, backSpace;
    private VuforiaLocalizer.Parameters parameters;
    private List<VuforiaTrackable> allTrackables;

    private OpenGLMatrix blueRoverLocationOnField, redFootprintLocationOnField, frontCratersLocationOnField, backSpaceLocationOnField;
    private OpenGLMatrix phoneLocationOnRobot;

    private Runnable visionProcess;
    private Thread visionThread;

    private Telemetry telemetry;

    /**
     * If you are standing in the Red Alliance Station looking towards the center of the field,
     * - The X axis runs from your left to the right. (positive from the center to the right)
     * - The Y axis runs from the Red Alliance Station towards the other side of the field
     * where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
     * - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
     * <p>
     * Robot:
     * FORWARD = +X
     * LEFT = +Y
     * UP = +Z
     */
    public Vision(int cameraMonitorViewId, ElapsedTime timer) {
        this.timer = timer;

        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565,true);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

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
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 0 : -180, 0, 0));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        vuforia.setFrameQueueCapacity(6);
        vuforia.enableConvertFrameToBitmap();
        frameQueue = vuforia.getFrameQueue();

        visionProcess = new Runnable() {
            public void run() {
                while (!Thread.currentThread().isInterrupted()) {
                    if (vuforiaActive) {
                        double[] location = localize();
                        x = location[0];
                        y = location[1];
                        hdg = location[2];
                    }
                    if (cvActive) {
                        Mat rgb = vuforiaToMat();
                        if(rgb != null) {
                            Mat gray = new Mat();
                            Imgproc.cvtColor(rgb, gray, Imgproc.COLOR_RGB2GRAY);
                            mineralVision.processFrame(rgb, gray);
                            goldPosition = mineralVision.getGoldPos();
                        }
                    }
                }
            }
        };
    }

    public void init() {
        visionThread = new Thread(visionProcess);
        visionThread.start();
    }

    public void stop() {
        visionThread.interrupt();
    }

    public void setActiveVuforia(boolean active) {
        vuforiaActive = active;
        if (active) {
            targetsRoverRuckus.activate();
        } else {
            targetsRoverRuckus.deactivate();
        }
    }

    public void setActiveCv(boolean active) {
        cvActive = active;
    }

    //x, y, hdg
    private double[] localize() {
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

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            return new double[] {translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, rotation.thirdAngle, timer.milliseconds()};
        }
        return new double[] {0, 0, 0, -1};
    }

    private Mat vuforiaToMat() {
        // returns empty Mat if correct image format not found or queue empty
        // grab frames and process them
        if (!frameQueue.isEmpty()) {
            VuforiaLocalizer.CloseableFrame vuforiaFrame = null;
            try {
                vuforiaFrame = frameQueue.take();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            if (vuforiaFrame != null) {
                Image rgb = vuforiaFrame.getImage(0);
                for (int i = 0; i < vuforiaFrame.getNumImages(); i++) {
                    rgb = vuforiaFrame.getImage(i);
                    if (rgb.getFormat() == PIXEL_FORMAT.RGB565) {
                        break;
                    }
                }
                Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
                bm.copyPixelsFromBuffer(rgb.getPixels());
                //put the image into a MAT for OpenCV
                Mat tmp = new Mat(rgb.getWidth(), rgb.getHeight(), CvType.CV_8UC4);
                Utils.bitmapToMat(bm, tmp);
                vuforiaFrame.close();
                return tmp;
            }
        } else {
            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        return new Mat(480, 854, CvType.CV_8UC4);
    }

    public double[] getLocation() {
        return new double[] {x, y, hdg};
    }
}