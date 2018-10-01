package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Log;

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
        try {
            contourVision = new MineralVisionContour();
            contourVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
            contourVision.setShowCountours(true);
            contourVision.enable();
            houghVision = new MineralVisionHough();
            houghVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
            houghVision.setShowCountours(true);
            houghVision.enable();
        } catch (Exception e)
        {
            Log.e("VisionTest", "STACKTRACE");
            Log.e("VisionTest", Log.getStackTraceString(e));
        }
    }

    public void loop() {
        try {
            contourVision.setShowCountours(gamepad1.a);
            houghVision.setShowCountours(gamepad1.b);
            telemetry.addData("goldPosContour: ", contourVision.getGoldPos());
            telemetry.addData("goldPosHough: ", houghVision.getGoldPos());
        } catch (NullPointerException e) {
            telemetry.addData("lineNo", e.getStackTrace()[0].getLineNumber());
            telemetry.addData("file", e.getStackTrace()[0].getFileName());
        }
        telemetry.update();
    }

    public void stop() {
        contourVision.disable();
        houghVision.disable();
    }
}
