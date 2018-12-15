package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@Autonomous(name = "Missing Chromosomes 2", group = "ThisIsWhatARTWouldDo")
public class DepressionAuto2 extends LinearOpMode {
    private Robot robot;
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        robot.drivetrain.setupIMU();
        robot.init();
        waitForStart();
        robot.drivetrain.driveEnc(48 * Math.sqrt(2));
        while (robot.drivetrain.isBusy() && opModeIsActive()) {}
        double startTime = getRuntime();
        robot.intake.intake(-0.8);
        while(opModeIsActive() && getRuntime() < startTime + 3) {}
        robot.intake.intake(0);
        robot.drivetrain.turn(90);
        while (robot.drivetrain.isBusy() && opModeIsActive()) {}
        robot.drivetrain.driveEnc(15);
        while (robot.drivetrain.isBusy() && opModeIsActive()) {}
        robot.drivetrain.turn(45);
        while (robot.drivetrain.isBusy() && opModeIsActive()) {}
        robot.drivetrain.driveEnc(76);
        robot.arm.extendDist(5);
        robot.arm.swingAngle(15);
        while (robot.drivetrain.isBusy() && opModeIsActive()) {}
        robot.arm.swingAngle(0);
        robot.stop();
    }
}
