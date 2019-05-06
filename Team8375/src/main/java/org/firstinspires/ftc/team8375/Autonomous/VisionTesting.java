package org.firstinspires.ftc.team8375.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.team8375.OpenCVClass;
import org.firstinspires.ftc.team8375.Subsystems.PIDModular;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "openCVTest", group = "Test")
public class VisionTesting extends LinearOpMode {
    private PIDModular pid;
    private OpenCVClass opencv;
    double a = Math.sqrt(Math.pow(2, 2) + Math.pow(2, 2));
    double b = Math.sqrt(2);
    double c = Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
    double heading = Math.acos(a/c);
    protected int x;
    protected int y;
    private double power = 0.5;

    //cubePos is from left to right -- 1 = left side, 2 = middle, 3 = right side
    protected int cubePos;
    protected List<Moments> mu;

    @Override
    public void runOpMode() throws InterruptedException {
        opencv = new OpenCVClass();
        opencv.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 1);
        opencv.enable();

        waitForStart();

        while(opModeIsActive()) {
            findCubePos(opencv.getContours(), true);

            //Imgproc.line(opencv.getRgb(), new Point(x, y), new Point(x, y), new Scalar(0, 255, 0), 5);

//            if (cubePos == 1) {
//                pid.run(-1, -1, -1, power, heading, 10, AxesOrder.ZYX);
//            } else if(cubePos == 2) {
//                pid.run(-1, -1, -1, power, 0, 10, AxesOrder.ZYX);
//            } else if(cubePos == 3) {
//                pid.run(-1, -1, -1, power, -heading, 10, AxesOrder.ZYX);
//            }

        }
        opencv.disable();
    }


    public void findCubePos(List<MatOfPoint> contours, boolean telemetryOn) {
        mu = new ArrayList<>(contours.size());
        for(int i = 0; i < contours.size(); i++){
            mu.add(i, Imgproc.moments(contours.get(i), false));
            Moments p = mu.get(i);
            x = (int) (p.get_m10() / p.get_m00());
            y = (int) (p.get_m01() / p.get_m00());
            // Imgproc.circle(mat, new Point(x, y), 4, new Scalar(49,0,255));
            if(y > 0 && y < 315){
                cubePos = 1;
            } else if(y > 300 && y < 630){
                cubePos = 2;
            } else if(y > 600 && y < 950){
                cubePos = 3;
            }
            else { cubePos = 0; }

            if(telemetryOn) {
                telemetry.addData("contours", Integer.toString(i));
                telemetry.addData("blobX", x);
                telemetry.addData("blobY", y);
                telemetry.addData("cubePos", cubePos);
                telemetry.update();
            }

        }
        contours.clear();



    }

}
