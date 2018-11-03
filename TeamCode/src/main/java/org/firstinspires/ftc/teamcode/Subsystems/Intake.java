package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private final double closePos = 1.0; //placeholders
    private final double openPos = 0.0;
    private DcMotor intake;
    private Servo door;
    public Intake(DcMotor intake, Servo door) {
        this.intake = intake;
        this.door = door;
    }

    public void intake(double power) {
        intake.setPower(power);
    }

    public void stop() {
        intake.setPower(0);
    }

    public void door(boolean open) {
        door.setPosition(open ? 0.75 : 0.25);
    }

}
