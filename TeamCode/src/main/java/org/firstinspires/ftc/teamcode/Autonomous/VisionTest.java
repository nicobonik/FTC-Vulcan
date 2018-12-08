package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.Subsystems.MineralVisionContour;
import org.firstinspires.ftc.teamcode.Subsystems.MineralVisionHough;

@Autonomous(name="VisionTest", group="Testing")
public class VisionTest extends OpMode {
    private MineralVisionContour contourVision;
    private boolean contour = true;
    private boolean hough = false;
    public void init() {
        contourVision = new MineralVisionContour();
        contourVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 1); //add a third parameter with value 1 to use front camera
        contourVision.setShowCountours(true);
        contourVision.setTelemetry(telemetry);
        contourVision.enable();
    }

    public void loop() {
        telemetry.addData("gold", contourVision.getGoldPos());
        telemetry.addData("x", contourVision.x);
        telemetry.addData("y", contourVision.y);
        telemetry.addData("range", contourVision.imageHeight / 2 - 20 + " - " + contourVision.imageHeight / 2 + 20);
        telemetry.addData("hls", "h: " + contourVision.h + ", l: " + contourVision.l + ", s: " + contourVision.s);
        telemetry.update();
    }

    public void stop() {
        contourVision.disable();
    }
}
