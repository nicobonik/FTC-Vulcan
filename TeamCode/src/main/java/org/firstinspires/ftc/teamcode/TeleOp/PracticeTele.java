package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Practice", group="Drive")
public class PracticeTele extends OpMode {
    boolean last;

    public void init() {
        last = false;
    }

    public void loop() {
        if(gamepad1.a) {
            last = true;
        }
    }

    public void stop() {
    }
}