package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.Subsystems.MineralVisionContour;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@Autonomous(name = "the robot is literally dying", group = "ThisIsWhatARTWouldDo")
public class VeryBasicAuto extends LinearOpMode {
    private Robot robot;
    private MineralVisionContour vis;
    protected boolean reSweep;
    protected int sweepCount = 0;
    protected int sweepProgress = 0;
    public void runOpMode() {
        vis = new MineralVisionContour();
        vis.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 1);
        vis.setShowCountours(true);
        vis.enable();
        robot = new Robot(hardwareMap, telemetry);
        robot.init();
        robot.drivetrain.setupIMU();
        waitForStart();
        telemetry.addData("started", "true");
        telemetry.update();
        robot.drivetrain.driveEnc(6);
        robot.arm.swingAngle(70);
        while((robot.drivetrain.isBusy() || robot.arm.isBusy()) && opModeIsActive()) {}
        robot.drivetrain.turn(45);
        while (robot.drivetrain.isBusy() && opModeIsActive()) {}
        runUntilGold();
        robot.drivetrain.driveEnc(12);
        while (robot.drivetrain.isBusy() && opModeIsActive()) {}
        robot.drivetrain.turn(-2 * robot.drivetrain.heading());
        while (robot.drivetrain.isBusy() && opModeIsActive()) {}
        robot.drivetrain.driveEnc(12 * Math.sqrt(2));
        while (robot.drivetrain.isBusy() && opModeIsActive()) {}
        robot.drivetrain.turn(-robot.drivetrain.heading());
        while (robot.drivetrain.isBusy() && opModeIsActive()) {}
        robot.drivetrain.driveEnc(12);
        while (robot.drivetrain.isBusy() && opModeIsActive()) {}
        double startTime = getRuntime();
        robot.intake.intake(-0.8);
        while(opModeIsActive() && getRuntime() < startTime + 3) {}
        robot.intake.intake(0);
    }

    private void runUntilGold() {
        while (!vis.getGoldPos()) {
            robot.drivetrain.turn(reSweep ? 5 : -5);
            sweepProgress++;
            if(sweepProgress > 18) {
                reSweep = !reSweep;
                sweepProgress = 0;
                sweepCount++;
                if(sweepCount > 1) {
                    robot.drivetrain.turn(-robot.drivetrain.heading());
                }
            }
            while (robot.drivetrain.isBusy() && opModeIsActive()) {}
        }
        int counter = 0;
        for(int i = 0; i < 10; i++) {
            if(vis.getGoldPos()) {
                counter++;
            }
        }
        if(counter < 3) {
            runUntilGold();
        }
    }
}
