package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@Autonomous(name = "AutonomousProgramForTimesOfGreatDarknessAndDesperation", group = "ThisIsWhatARTWouldDo")
public class AutonomousProgramForTimesOfGreatDarknessAndDesperation extends LinearOpMode {
    private Robot robot;
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        robot.drivetrain.setupIMU();
        robot.init();
        waitForStart();
        robot.arm.swingAngle(90);
        robot.arm.extendDist(8);
        robot.drivetrain.driveEnc(24 * Math.sqrt(2));
        while (robot.drivetrain.isBusy() && opModeIsActive()) {}
        robot.arm.swingAngle(0);
        while (robot.arm.isBusy() && opModeIsActive()) {}
        robot.stop();
    }
}
