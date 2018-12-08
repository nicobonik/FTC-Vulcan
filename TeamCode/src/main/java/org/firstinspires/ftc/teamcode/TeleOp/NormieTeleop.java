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
    private boolean lastBump, lastLeft, lastRight, lastUp, lastDown;

    public void init() {
        robot = new Robot(hardwareMap, telemetry);
    }

    public void start() {
        robot.init();
        robot.drivetrain.setupIMU();
        robot.drivetrain.setZeroP(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void loop() {
        //emergency stop
        if(gamepad1.guide || gamepad2.guide) {
            robot.disableServos();
            stop();
        }
        //drivetrain
        if(gamepad1.right_trigger > 0.55) {
            robot.drivetrain.tempPower = ((Drivetrain.BASE_POWER / 4));
        } else {
            robot.drivetrain.tempPower = Drivetrain.BASE_POWER;
        }
        robot.drivetrain.setGamepadState(scale(-gamepad1.left_stick_y), scale(gamepad1.left_stick_x), scale(gamepad1.right_stick_x));
        //arm
        if(gamepad1.a || gamepad1.a) {
            robot.arm.setZeroP(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        if(gamepad1.b || gamepad2.b) {
            robot.arm.setZeroP(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if(gamepad2.y) {
            robot.arm.swingAngle(90);
        } else if (gamepad2.x) {
            robot.arm.swingAngle(0);
        } else {
            robot.arm.swing(scale(-gamepad2.left_stick_y));
        }
        if(gamepad2.dpad_up && !lastUp) {
            robot.arm.extend(true);
        } else if(gamepad2.dpad_down && !lastDown) {
            robot.arm.extend(false);
        } else {
            robot.arm.extend(scale(-gamepad2.right_stick_y));
        }
        //intake
        if(gamepad2.right_trigger > 0.2) {
            robot.intake.intake(-0.8);
        } else if(lastRight) {
            robot.intake.intake(0);
        } else if(gamepad2.left_trigger > 0.2 && !lastLeft) {
            robot.intake.toggleIntake();
        }
        if(gamepad2.left_bumper) {
            robot.intake.door(true);
        } else if(gamepad2.right_bumper && !lastBump) {
            robot.intake.door(false);
        }
        lastBump = gamepad2.right_bumper;
        lastLeft = gamepad2.left_trigger > 0.2;
        lastRight = gamepad2.right_trigger > 0.2;
        lastUp = gamepad2.dpad_up;
        lastDown = gamepad2.dpad_down;

        telemetry.addData("armPower", robot.arm.swingPower);
        telemetry.addData("arm", robot.arm.arm[1].getCurrentPosition());
        telemetry.addData("extend target", robot.arm.extendPosition);
    }

    public void stop() {
        robot.stop();
    }

    private double scale(double input) {
        return (input / 0.7) * (0.3 * Math.pow(input, 6) + 0.4);
    }
}
