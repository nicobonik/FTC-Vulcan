package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

@TeleOp(name="NormieDrive", group="Drive")
public class NormieTeleop extends OpMode {
    protected Robot robot;
    private boolean lastY;

    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        gamepad1.setJoystickDeadzone(0.05f);
    }

    public void start() {
        robot.init();
        robot.drivetrain.setupIMU();
    }

    public void loop() {
        //drivetrain
        if(gamepad1.right_trigger > 0.55) {
            robot.drivetrain.tempPower = ((Drivetrain.BASE_POWER / 4));
        } else {
            robot.drivetrain.tempPower = Drivetrain.BASE_POWER;
        }
        robot.drivetrain.setGamepadState(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        //arm
        if(gamepad2.y) {
            robot.arm.swing(true);
        } else if (gamepad2.x) {
            robot.arm.swing(false);
        } else {
            robot.arm.swing(-gamepad2.left_stick_y);
            robot.arm.extend(-gamepad2.right_stick_y);
        }
        //intake
        if(gamepad2.a) {
            robot.intake.intake(0.8);
        } else if(gamepad2.b) {
            robot.intake.intake(-0.8);
        } else {
            robot.intake.intake(0);
        }
        if(gamepad1.x) {
            robot.intake.door(true);
        } else if(gamepad1.y && !lastY) {
            robot.intake.door(false);
        }
        lastY = gamepad1.y;
    }

    public void stop() {
        robot.stop();
    }
}
