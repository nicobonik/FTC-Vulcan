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
    private Robot robot;
    private MineralVisionContour vis;
    public void runOpMode() {
        vis = new MineralVisionContour();
        vis.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 1);
        vis.setShowCountours(true);
        vis.enable();
        robot = new Robot(hardwareMap, telemetry);
        robot.drivetrain.setupIMU();
        robot.init();
        waitForStart();
        robot.drivetrain.driveEnc(6);
        while(robot.drivetrain.isBusy() && opModeIsActive()) {}
        sleep(2000);
        robot.drivetrain.turnEnc(50);
        while(robot.drivetrain.isBusy() && opModeIsActive()) {}
        while (!vis.getGoldPos() && opModeIsActive()) {
            telemetry.addData("gold", "not found");
            telemetry.update();
        }
        telemetry.addData("gold", "found");
        telemetry.update();
        vis.disable();
        robot.drivetrain.driveEnc(6);
        while(robot.drivetrain.isBusy() && opModeIsActive()) {}
        robot.stop();
    }
}