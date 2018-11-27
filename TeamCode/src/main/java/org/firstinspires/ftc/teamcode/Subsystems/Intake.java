package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends Subsystem {
    private final double closePos = 1.0; //placeholders
    private final double openPos = 0.0;
    public volatile int position;
    private volatile double power;
    private CRServo intake;
    private Servo door;
    private Thread systemThread;
    public Intake(CRServo in, Servo dr) {
        intake = in;
        door = dr;
        power = 0;
        position = 0;
    }

    public void updateSubsystem() {
        intake.setPower(power);
        door.setPosition((position * 0.4) + 0.15);
    }

    public void intake(double power) {
        this.power = power;
    }

    public void stop() {
        intake.setPower(0);
        door.setPosition(0.75);
    }

    public void door(boolean closed) {
        if(closed) {
            position = 0;
        } else {
            position = Math.min(++position, 2);
        }
    }
}
