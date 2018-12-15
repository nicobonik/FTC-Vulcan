package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@Autonomous(name = "Missing Chromosomes", group = "ThisIsWhatARTWouldDo")
public class DepressionAutoDepot extends LinearOpMode {
    private Robot robot;
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        robot.drivetrain.setupIMU();
        robot.init();
        robot.drivetrain.driveEnc(48 * Math.sqrt(2));
        while (robot.drivetrain.isBusy() && opModeIsActive()) {}
        while(robot.arm.isBusy() && opModeIsActive()) {}
        robot.stop();
    }
}
