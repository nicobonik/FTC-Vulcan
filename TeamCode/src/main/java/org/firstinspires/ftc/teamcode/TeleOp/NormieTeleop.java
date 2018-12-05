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
    private boolean lastBump;

    public void init() {
        robot = new Robot(hardwareMap, telemetry);
    }

    public void start() {
        robot.init();
        robot.drivetrain.setupIMU();
    }

    public void loop() {
        //emergency stop
        if(gamepad1.guide || gamepad2.guide) {
            stop();
        }
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
        }//intake
        if(gamepad2.a) {
            robot.intake.intake(0.8);
        } else if(gamepad2.b) {
            robot.intake.intake(-0.8);
        } else {
            robot.intake.intake(0);
        }
        if(gamepad2.left_bumper) {
            robot.intake.door(true);
        } else if(gamepad2.right_bumper && !lastBump) {
            robot.intake.door(false);
        }
        lastBump = gamepad2.right_bumper;
        robot.drivetrain.setGamepadState(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        telemetry.addData("loop", "completed");
    }

    public void stop() {
        robot.stop();
    }
}
