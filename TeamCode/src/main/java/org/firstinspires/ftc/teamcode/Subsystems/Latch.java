package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class Latch {
    Servo hook;
    public Latch(Servo hook) {
        this.hook = hook;
    }
    public void latch(Boolean lock) {
        hook.setPosition(lock ? 0.0 : 1.0);
    }
}
