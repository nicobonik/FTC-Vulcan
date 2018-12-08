package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Vision;

@Autonomous(name = "NavTest", group = "Testing")
public class NavigationTest extends LinearOpMode {
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        /*Vision nav = new Vision(cameraMonitorViewId, telemetry);
        nav.setActiveVuforia(false);
        nav.setActiveCv(true);
        nav.init();
        sleep(10000);
        while(opModeIsActive()) {}
        nav.stop();*/
    }
}
