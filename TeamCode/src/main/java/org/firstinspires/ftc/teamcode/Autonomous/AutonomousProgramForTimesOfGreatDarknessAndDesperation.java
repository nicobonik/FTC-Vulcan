package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@Autonomous(name = "AutonomousProgramForTimesOfGreatDarknessAndDesperation", group = "ThisIsWhatARTWouldDo")
public class AutonomousProgramForTimesOfGreatDarknessAndDesperation extends LinearOpMode {
    private Robot robot;
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        robot.drivetrain.setupIMU();
        robot.init();
        while(!isStarted()) {
            robot.arm.swingAngle(0);
        }
        telemetry.addData("start", "started");
        telemetry.update();
        robot.arm.extend(2);
        robot.arm.setZeroP(DcMotor.ZeroPowerBehavior.FLOAT);
        while(robot.arm.isBusy() && opModeIsActive()) {}
        robot.arm.setZeroP(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.arm.swingAngle(90);
        robot.arm.extendDist(8);
        while(robot.arm.isBusy() && opModeIsActive()) {}
        robot.drivetrain.driveEnc(10 * Math.sqrt(2));
        while (robot.drivetrain.isBusy() && opModeIsActive()) {}
        robot.arm.swingAngle(0);
        robot.arm.extendDist(0);
        robot.drivetrain.turn(90);
        while (robot.drivetrain.isBusy() && opModeIsActive()) {}
        robot.drivetrain.driveEnc(36 * Math.sqrt(2));
        while (robot.drivetrain.isBusy() && opModeIsActive()) {}
        robot.drivetrain.turn(45);
        while (robot.drivetrain.isBusy() && opModeIsActive()) {}
        robot.drivetrain.driveEnc(36);
        while (robot.drivetrain.isBusy() && opModeIsActive()) {}
        double startTime = getRuntime();
        robot.intake.intake(-0.8);
        while(opModeIsActive() && getRuntime() < startTime + 3) {}
        robot.intake.intake(0);
        robot.drivetrain.driveEnc(-64);
        robot.arm.extendDist(0);
        while ((robot.arm.isBusy() || robot.drivetrain.isBusy()) && opModeIsActive()) {}
        robot.stop();
    }
}
