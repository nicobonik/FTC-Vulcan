package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@TeleOp(name="BasicDrive", group="Drive")
public class BasicTele extends OpMode {
    protected Drivetrain drivetrain;

    public void init() {
        drivetrain = new Drivetrain(
                hardwareMap.dcMotor.get("front_left"),
                hardwareMap.dcMotor.get("front_right"),
                hardwareMap.dcMotor.get("back_left"),
                hardwareMap.dcMotor.get("back_right")
        );
    }

    public void loop() {
        if(gamepad2.a) {
            drivetrain.arcadeDrive(gamepad2.left_stick_y, gamepad2.right_stick_x);
        } else {
            drivetrain.mecanumDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 0.8);
        }
    }

    public void stop() {
        drivetrain.stop();
    }
}
