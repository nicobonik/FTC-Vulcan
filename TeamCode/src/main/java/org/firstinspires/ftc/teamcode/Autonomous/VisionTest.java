package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.Subsystems.MineralVisionContour;
import org.firstinspires.ftc.teamcode.Subsystems.MineralVisionHough;

@Autonomous(name="VisionTest", group="Vision")
public class VisionTest extends OpMode {
    private MineralVisionContour contourVision;
    private MineralVisionHough houghVision;
    private boolean contour = true;
    private boolean hough = false;
    public void init() {
        if(contour) {
            contourVision = new MineralVisionContour();
            contourVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
            contourVision.setShowCountours(true);
            contourVision.enable();
        } else if(hough) {
            houghVision = new MineralVisionHough();
            houghVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
            houghVision.setShowCountours(true);
            houghVision.setTelem(telemetry);
            houghVision.enable();
        }
    }

    public void loop() {
        if(contour) {
            telemetry.addData("GoldPos", contourVision.getGoldPos());
        } else if(hough) {
            telemetry.addData("GoldPos", houghVision.getGoldPos());
        }
    }

    public void stop() {
        if(contour) {
            contourVision.disable();
        } else if(hough) {
            houghVision.disable();
        }
    }
}
