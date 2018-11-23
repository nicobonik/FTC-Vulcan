package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

@TeleOp(name="NormieDrive", group="Drive")
public class NormieTeleop extends OpMode {
    protected Robot robot;

    public void init() {
        telemetry.addData("robot", "initializing");
        telemetry.update();
        robot = new Robot(hardwareMap, telemetry);
        telemetry.addData("robot", "initialized");
        telemetry.update();
        //gamepad1.setJoystickDeadzone(0.05f);
    }

    public void start() {
        //robot.init();
    }

    public void loop() {
        telemetry.addData("loop", "started");
        telemetry.update();
        if(gamepad1.right_trigger > 0.55) {
            robot.drivetrain.tempPower = ((Drivetrain.BASE_POWER / 4));
        } else {
            robot.drivetrain.tempPower = Drivetrain.BASE_POWER;
        }
        telemetry.addData("slow", "checked");
        telemetry.update();
        if(gamepad1.a) {
            robot.intake.intake(1);
        } else if(gamepad1.b) {
            robot.intake.intake(-1);
        } else {
            robot.intake.intake(0);
        }
        telemetry.addData("intake", "checked");
        telemetry.update();
        if(gamepad1.x) {
            robot.intake.door(true);
        } else if(gamepad1.y) {
            robot.intake.door(false);
        }
        telemetry.addData("door", "checked");
        telemetry.update();
        /*double dy = gamepad2.left_stick_y - y;
        y = Range.clip(y + Range.clip(dy/20, -0.05, 0.05), -1.0, 1.0);
        arm.swing(y);*/
        //robot.drivetrain.setGamepadState(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        for (int i = 1; i < 3; i++) {
            telemetry.addData("subsystem", "updating");
            telemetry.update();
            if (robot.subsystems[i] != null) {
                robot.subsystems[i].updateSubsystem();
            }
            telemetry.addData("subsystem", "updated");
            telemetry.update();
        }

        double vd = Math.hypot((-gamepad1.left_stick_y / 0.7) * (0.3 * Math.pow(-gamepad1.left_stick_y, 6) + 0.4), (gamepad1.left_stick_x / 0.7) * (0.3 * Math.pow(gamepad1.left_stick_x, 6) + 0.4));
        double theta = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - (Math.PI / 4);
        double[] v = {
                vd * Math.sin(theta) + gamepad1.right_stick_x,
                vd * Math.cos(theta) - gamepad1.right_stick_x,
                vd * Math.cos(theta) + gamepad1.right_stick_x,
                vd * Math.sin(theta) - gamepad1.right_stick_x
        };
        robot.drivetrain.speeds(v);
    }

    public void stop() {
        robot.stop();
    }
}
