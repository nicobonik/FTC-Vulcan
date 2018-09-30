package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeLift {
    private final double closePos = 1.0; //placeholders
    private final double openPos = 0.0;
    private int position = 0;
    private DcMotor[] arm;
    private DcMotor intake;
    private CRServo intake1;
    private CRServo intake2;
    private Servo hook;
    private Servo door;
    public IntakeLift(DcMotor[] arm, DcMotor intake, Servo door) {
        this.arm = arm;
        this.intake = intake;
        //this.hook = hook;
        this.door = door;
        for (int i = 0; i < 3; i++) {
            this.arm[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.arm[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.arm[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public IntakeLift(DcMotor[] arm, CRServo intake1, CRServo intake2, Servo door) {
        this.arm = arm;
        this.intake1 = intake1;
        this.intake2 = intake2;
        //this.hook = hook;
        this.door = door;
        for (int i = 0; i < 3; i++) {
            this.arm[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.arm[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.arm[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void intake(int direction) {
        //intake.setPower(direction);
        intake1.setPower(direction);
        intake2.setPower(-direction);
    }

    public void stop() {
        for (int i = 0; i < 3; i++) {
            arm[i].setPower(0);
        }
        intake1.setPower(0);
        intake2.setPower(0);
    }

    public void swing(double power) {
        for (int i = 0; i < 3; i++) {
            arm[i].setTargetPosition(position + (int)((power / 0.7) * (0.3 * Math.pow(power, 6) + 0.4) * 10));
            arm[i].setPower(1.0);
        }
    }

    public void door(boolean open) {
        door.setPosition(open ? 0.75 : 0.25);
    }

    public void latch(Boolean lock) {
        hook.setPosition(lock ? closePos : openPos);
    }
}
