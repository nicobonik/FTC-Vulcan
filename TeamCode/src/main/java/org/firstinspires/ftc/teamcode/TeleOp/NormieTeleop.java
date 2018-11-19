package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@TeleOp(name="NormieDrive", group="Drive")
public class NormieTeleop extends OpMode {
    protected Robot robot;

    public void init() {
        robot = new Robot(hardwareMap);
        gamepad1.setJoystickDeadzone(0.05f);
    }

    public void start() {
        robot.drivetrain.init();
        robot.intake.init();
        robot.arm.init();
    }

    public void loop() {
        if(gamepad1.right_trigger > 0.55) {
            robot.drivetrain.tempPower = ((Drivetrain.BASE_POWER / 4));
        } else {
            robot.drivetrain.tempPower = Drivetrain.BASE_POWER;
        }
        if(gamepad1.a) {
            robot.intake.intake(1);
        } else if(gamepad1.b) {
            robot.intake.intake(-1);
        } else {
            robot.intake.intake(0);
        }
        if(gamepad1.x) {
            robot.intake.door(true);
        } else if(gamepad1.y) {
            robot.intake.door(false);
        }
        /*double dy = gamepad2.left_stick_y - y;
        y = Range.clip(y + Range.clip(dy/20, -0.05, 0.05), -1.0, 1.0);
        arm.swing(y);*/
        robot.drivetrain.setGamepadState(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }

    public void stop() {
        robot.stop();
    }
}
