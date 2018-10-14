package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@Disabled
@TeleOp(name="ServoTest", group="Drive")
public class ServoTest extends OpMode {
    private CRServo intake;
    public void init() {
        intake = hardwareMap.crservo.get("intake");
    }

    public void loop() {
        intake.setPower(gamepad1.left_stick_y);
    }

    public void stop() {
        intake.setPower(0);
    }
}