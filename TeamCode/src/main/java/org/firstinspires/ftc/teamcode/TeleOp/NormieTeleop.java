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
    private boolean lastTrig, lastA, lastB, lastUp, lastDown;

    public void init() {
        robot = new Robot(hardwareMap, telemetry);
    }

    public void start() {
        robot.init();
        robot.drivetrain.setupIMU();
        lastTrig = false;
        lastA = false;
        lastB = false;
        lastUp = false;
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
            robot.arm.swing(scale(-gamepad2.left_stick_y));
        }
        if(gamepad2.dpad_up) {
            robot.arm.extend(true);
        } else if (gamepad2.dpad_down) {
            robot.arm.extend(false);
        } else {
            robot.arm.extend(scale(-gamepad2.right_stick_y));
        }

        //intake
        if(gamepad2.b) {
            robot.intake.intake(-0.8);
        } else if(lastB) {
            robot.intake.intake(0);
        } else if(gamepad2.a && !lastA) {
            robot.intake.toggleIntake();
        }
        if(gamepad2.left_bumper) {
            robot.intake.door(true);
        } else if(gamepad2.right_trigger > 0.2 && !lastTrig) {
            robot.intake.door(false);
        }
        lastTrig = gamepad2.right_trigger > 0.2;
        lastA = gamepad2.a;
        lastB = gamepad2.b;
        lastUp = gamepad2.dpad_up;
        lastDown = gamepad2.dpad_down;
        robot.drivetrain.mecanumDrive(scale(gamepad1.left_stick_y), scale(gamepad1.left_stick_x), scale(gamepad1.right_stick_x), 0.8);
        telemetry.addData("loop", "completed");
    }

    public void stop() {
        robot.stop();
    }

    private double scale(double input) {
        return (input / 0.7) * (0.3 * Math.pow(input, 6) + 0.4);
    }
}
