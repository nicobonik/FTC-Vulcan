package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@TeleOp(name="NormieDrive", group="Drive")
public class NormieTeleop extends OpMode {
    protected Robot robot;
    protected Arm arm;
    public void init() {
        robot = new Robot(hardwareMap);
        arm = new Arm(hardwareMap);
    }

    public void start() {
        robot.drivetrain.setupIMU();
    }

    public void loop() {
        robot.drivetrain.mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 0.8);
        arm.setPowers(-gamepad2.left_stick_y, -gamepad2.right_stick_y, 500);
        //test stuffs
        telemetry.addData("encoderTicks = ", arm.lift.getCurrentPosition());
    }

    public void stop() { robot.stop(); }
}
