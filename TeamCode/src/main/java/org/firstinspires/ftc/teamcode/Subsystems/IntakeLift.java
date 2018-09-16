package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeLift {
    private final double closePos = 1.0; //placeholders
    private final double openPos = 0.0;
    private DcMotor arm;
    private DcMotor intake;
    private Servo hook;
    private Servo door;
    public IntakeLift(DcMotor arm, DcMotor intake, Servo hook) {
        this.arm = arm;
        this.intake = intake;
        this.hook = hook;
        //this.door = door;
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void intake(Boolean reverse) {
        intake.setPower(reverse ? -1.0 : 1.0);
    }

    public void stop() {
        arm.setPower(0);
        intake.setPower(0);
    }

    public void swing(double power) {
        arm.setPower(power);
    }

    public void latch(Boolean lock) {
        hook.setPosition(lock ? closePos : openPos);
    }
}
