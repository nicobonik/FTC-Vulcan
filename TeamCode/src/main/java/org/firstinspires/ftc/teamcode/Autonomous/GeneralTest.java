package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Subsystems.MineralVisionContour;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@Autonomous(name = "GeneralTest", group = "Testing")
public class GeneralTest extends LinearOpMode {
    private Robot robot;
    private MineralVisionContour vis;
    public void runOpMode() {
        /*vis = new MineralVisionContour();
        vis.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 1);
        vis.setShowCountours(true);
        vis.enable();*/
        robot = new Robot(hardwareMap, telemetry);
        robot.drivetrain.setupIMU();
        robot.init();
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("position", robot.drivetrain.getPosition());
            telemetry.update();
        }
        robot.stop();
    }
}