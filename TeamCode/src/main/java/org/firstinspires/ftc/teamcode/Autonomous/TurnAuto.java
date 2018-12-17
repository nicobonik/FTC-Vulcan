package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

public class TurnAuto extends LinearOpMode {
    private Robot robot;
    public void runOpMode() {
        robot = new Robot(hardwareMap);
        robot.drivetrain.setupIMU();
        waitForStart();
        robot.drivetrain.turn(90);
        robot.drivetrain.driveForwards(500);
        robot.stop();
    }
}
