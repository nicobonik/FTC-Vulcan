package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@TeleOp(name="PassengerDrive", group="Memes")
public class PassengerTeleOp extends OpMode {
    Drivetrain drivetrain;
    double y;
    double x;
    public void init() {
        drivetrain = new Drivetrain(hardwareMap.dcMotor.get("front_left"), hardwareMap.dcMotor.get("front_right"), hardwareMap.dcMotor.get("back_left"), hardwareMap.dcMotor.get("back_right"));
        drivetrain.setZeroP(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void start() {
        drivetrain.setZeroP(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void loop() {
        double dy = gamepad1.left_stick_y - y;
        double dx = gamepad1.right_stick_x - x;
        y += Range.clip(dy/20, -0.05, 0.05);
        x += Range.clip(dx/20, -0.05, 0.05);
        drivetrain.arcadeDrive(y, x);
    }

    public void stop() {
        drivetrain.stop();
    }
}
