package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeLift;

@TeleOp(name="NormieDrive", group="Drive")
public class NormieTeleop extends OpMode {
    protected Drivetrain drivetrain;
    protected IntakeLift inlift;
    private double y;

    public void init() {
        drivetrain = new Drivetrain(
                hardwareMap.dcMotor.get("front_left"),
                hardwareMap.dcMotor.get("front_right"),
                hardwareMap.dcMotor.get("back_left"),
                hardwareMap.dcMotor.get("back_right")
        );
        inlift = new IntakeLift(new DcMotor[] {hardwareMap.dcMotor.get("arm1"), hardwareMap.dcMotor.get("arm2"), hardwareMap.dcMotor.get("arm3")},
                hardwareMap.crservo.get("intake1"),
                hardwareMap.crservo.get("intake2"),
                hardwareMap.servo.get("door")
                );
        gamepad1.setJoystickDeadzone(0.05f);
        brake = true;
    }

    public void loop() {
        if(gamepad1.right_trigger > 0.55) {
            drivetrain.tempPower = ((Drivetrain.BASE_POWER / 4));
        } else {
            drivetrain.tempPower = Drivetrain.BASE_POWER;
        }
        if(gamepad1.a) {
            inlift.intake(1);
        } else if(gamepad1.b) {
            inlift.intake(-1);
        } else {
            inlift.intake(0);
        }
        if(gamepad1.x) {
            inlift.door(true);
        } else if(gamepad1.y) {
            inlift.door(false);
        }
        double dy = gamepad2.left_stick_y - y;
        y = Range.clip(y + Range.clip(dy/20, -0.05, 0.05), -1.0, 1.0);
        inlift.swing(y);
        drivetrain.arcadeDrive(gamepad1.left_stick_y, gamepad1.right_stick_x);

    }

    public void stop() {
        drivetrain.stop();
    }
}
