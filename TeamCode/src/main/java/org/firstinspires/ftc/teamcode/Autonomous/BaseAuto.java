package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.MineralVisionContour;
import org.firstinspires.ftc.teamcode.Subsystems.MineralVisionHough;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@Autonomous(name = "TestAuto", group="Auto")
public class BaseAuto extends LinearOpMode {
    protected Robot robot;
    protected MineralVisionContour vis;
    protected boolean reSweep;
    protected int sweepCount = 0;
    protected int sweepProgress = 0;
    public void runOpMode() {
        vis = new MineralVisionContour();
        vis.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 1);
        vis.setShowCountours(true);
        vis.enable();
        robot = new Robot(hardwareMap, telemetry);
        robot.drivetrain.setupIMU();
        robot.init();
        waitForStart();
        robot.arm.swingAngle(90);
        robot.arm.extendDist(8);
        robot.drivetrain.driveEnc(6);
        while(robot.drivetrain.isBusy() || robot.arm.isBusy() && opModeIsActive()) {}
        robot.arm.extendDist(0);
        while(robot.arm.isBusy() && opModeIsActive()) {}
    }

    private void runUntilGold() {
        while (!vis.getGoldPos()) {
            robot.drivetrain.turnEnc(reSweep ? 5 : -5);
            sweepProgress++;
            if(sweepProgress > 20) {
                reSweep = !reSweep;
                sweepProgress = 0;
            }
            while (robot.drivetrain.isBusy() && opModeIsActive()) {}
        }
        int counter = 0;
        for(int i = 0; i < 10; i++) {
            if(vis.getGoldPos()) {
                counter++;
            }
        }
        if(counter < 7) {
            runUntilGold();
        }
    }
}