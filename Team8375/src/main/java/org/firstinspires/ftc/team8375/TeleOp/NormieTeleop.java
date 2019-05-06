package org.firstinspires.ftc.team8375.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team8375.Subsystems.Arm;
import org.firstinspires.ftc.team8375.Subsystems.Robot;

@TeleOp(name="NormieDrive", group="Drive")
public class NormieTeleop extends OpMode {
    protected Robot robot;
   // protected Arm arm;
    public void init() {
        robot = new Robot(hardwareMap);
       // arm = new Arm(200, hardwareMap);
       // telemetry.addData("Encoder Value", arm.lift.getCurrentPosition());
       // gamepad1.setJoystickDeadzone(0.075f);
       // gamepad2.setJoystickDeadzone(0.075f);
    }

    public void start() {
        robot.drivetrain.setupIMU();
       // arm.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // arm.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.drivetrain.Time.reset();
    }

    public void loop() {
        robot.drivetrain.tankDrive(gamepad1.left_stick_y, -gamepad1.right_stick_x, 0.5, -1);
        //arm.setPowers(gamepad2.left_stick_y, -gamepad2.right_stick_y, gamepad2.left_trigger, gamepad2.a, 500, 2200, 400.0);
       // telemetry.addData("Arm Position", arm.lift.getCurrentPosition());
       // telemetry.addData("Time", robot.drivetrain.Time.time());
    }

    public void stop() {
        robot.stop();
    }
}
