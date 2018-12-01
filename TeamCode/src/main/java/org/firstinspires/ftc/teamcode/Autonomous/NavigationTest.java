package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Navigation;

@Autonomous(name = "NavTest", group = "Vision")
public class NavigationTest extends LinearOpMode {
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        Navigation nav = new Navigation(cameraMonitorViewId, telemetry);
        nav.setActiveVuforia(false);
        nav.setActiveCv(true);
        nav.init();
        sleep(10000);
        while(opModeIsActive()) {}
        nav.stop();
    }
}
