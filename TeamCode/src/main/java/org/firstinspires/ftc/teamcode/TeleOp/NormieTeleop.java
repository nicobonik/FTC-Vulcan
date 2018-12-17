package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@TeleOp(name="NormieDrive", group="Drive")
public class NormieTeleop extends OpMode {
    protected Robot robot;

    public void init() {
        robot = new Robot(hardwareMap);
    }

    public void start() {
        robot.drivetrain.setupIMU();
    }

    public void loop() {
        robot.drivetrain.mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 0.8);
    }

    public void stop() {
        robot.stop();
    }
}
