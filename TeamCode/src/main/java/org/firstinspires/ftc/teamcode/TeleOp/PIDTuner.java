package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

public class PIDTuner extends OpMode {
    private Robot robot;
    private double Kp, Ki, Kd;
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        Kp = 0;
        Ki = 0;
        Kd = 0;
    }

    public void loop() {
        if(gamepad1.a) {
            Kp += 0.01;
        }
        if(gamepad1.b) {
            Kp -= 0.01;
        }
        if(gamepad1.x) {
            Ki += 0.01;
        }
        if(gamepad1.y) {
            Ki -= 0.01;
        }
        if(gamepad1.left_bumper) {
            Kd += 0.01;
        }
        if(gamepad1.right_bumper) {
            Kd -= 0.01;
        }
        robot.drivetrain.drivePID.setCoefficients(Kp, Ki, Kd);
        telemetry.addData("kp: ", Kp);
        telemetry.addData("ki: ", Ki);
        telemetry.addData("kd: ", Kd);
        telemetry.update();
    }

    public void stop() {
        robot.stop();
    }
}
