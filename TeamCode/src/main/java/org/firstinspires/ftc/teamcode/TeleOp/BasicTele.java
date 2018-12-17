package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="BasicDrive", group="Drive")
public class BasicTele extends OpMode {
    DcMotor fl, fr, bl, br;

    public void init() {
        fl = hardwareMap.dcMotor.get("front_left");
        fr = hardwareMap.dcMotor.get("front_right");
        bl = hardwareMap.dcMotor.get("back_left");
        br = hardwareMap.dcMotor.get("back_right");
        fr.setDirection(DcMotor.Direction.REVERSE); //idk why I only have to invert this one, our motor directions are being wanky
    }

    public void loop() {
        double vd = Math.hypot((-gamepad1.left_stick_y / 0.7) * (0.3 * Math.pow(-gamepad1.left_stick_y, 6) + 0.4), (gamepad1.left_stick_x / 0.7) * (0.3 * Math.pow(gamepad1.left_stick_x, 6) + 0.4));
        double theta = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - (Math.PI / 4);
        double[] v = {
            vd * Math.sin(theta) + gamepad1.right_stick_x,
            vd * Math.cos(theta) - gamepad1.right_stick_x,
            vd * Math.cos(theta) + gamepad1.right_stick_x,
            vd * Math.sin(theta) - gamepad1.right_stick_x
        };
        fl.setPower(v[0]);
        fr.setPower(v[1]);
        bl.setPower(v[2]);
        br.setPower(v[3]);
    }

    public void stop() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}
