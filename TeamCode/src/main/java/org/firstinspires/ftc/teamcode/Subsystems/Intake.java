package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.LinkedHashMap;

public class Intake extends Subsystem {
    public volatile int position;
    private volatile double power;
    private CRServo intake;
    private Servo door;
    private double[] positions = {0.3, 0.55, 0.9};
    public Intake(CRServo in, Servo dr) {
        intake = in;
        in.setDirection(DcMotorSimple.Direction.REVERSE);
        door = dr;
        power = 0;
        position = 0;
    }

    public LinkedHashMap<String, String> updateSubsystem() {
        intake.setPower(power);
        door.setPosition(positions[position]);
        return telemetryPackets;
    }

    //todo: make intake toggle
    public void toggleIntake() {
        if(power != 0) {
            power = 0;
        } else {
            power = 0.8;
        }
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
