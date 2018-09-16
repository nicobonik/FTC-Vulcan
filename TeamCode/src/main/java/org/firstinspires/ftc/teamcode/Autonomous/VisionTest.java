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
    public void init() {
        contourVision = new MineralVisionContour();
        contourVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        contourVision.setShowCountours(true);
        contourVision.enable();
        houghVision = new MineralVisionHough();
        houghVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        houghVision.setShowCountours(true);
        houghVision.enable();
    }

    public void loop() {
        contourVision.setShowCountours(gamepad1.a);
        houghVision.setShowCountours(gamepad1.b);
        telemetry.addData("goldPosContour: ", contourVision.getGoldPos());
        telemetry.addData("goldPosHough: ", houghVision.getGoldPos());
        telemetry.update();
    }

    public void stop() {
        contourVision.disable();
        houghVision.disable();
    }
}
