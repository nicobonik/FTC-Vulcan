package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends Subsystem {
    private final double closePos = 1.0; //placeholders
    private final double openPos = 0.0;
    private volatile boolean running, open;
    private volatile double power;
    private DcMotor intake;
    private Servo door;
    private Thread systemThread;
    public Intake(DcMotor in, Servo dr) {
        this.intake = in;
        this.door = dr;
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        power = 0;
        open = false;
    }

    public void updateSubsystem() {
        intake.setPower(power);
        door.setPosition(open ? 0.75 : 0.25);
    }

    public void intake(double power) {
        this.power = power;
    }

    public void stop() {
        intake.setPower(0);
    }

    public void door(boolean open) {
        this.open = open;
    }

}
