package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeLift {
    private final double closePos = 1.0; //placeholders
    private final double openPos = 0.0;
    private final double baseArmLength = 16;
    private final double armAngleOffset = 5;
    private final double armHeight = 6;
    private final double omniRadius = 2;
    private final double extensionRadius = 1;
    private final double ticksPerRevolution = 1440;
    private int position = 0;
    private DcMotor[] arm;
    private DcMotor intake;
    private DcMotor extender;
    private CRServo intake1;
    private CRServo intake2;
    private Servo hook;
    private Servo door;
    public IntakeLift(DcMotor[] arm, DcMotor extender, DcMotor intake, Servo door) {
        this.arm = arm;
        this.extender = extender;
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
        int pos = Math.max(position + (int)((power / 0.7) * (0.3 * Math.pow(power, 6) + 0.4) * 10), (int)(Math.atan((armHeight - omniRadius) / (baseArmLength + ((extender.getCurrentPosition() * 2 * Math.PI * extensionRadius) / ticksPerRevolution))) * ticksPerRevolution / 360));
        pos = Math.min(pos, 0);
        for (int i = 0; i < 3; i++) {
            arm[i].setTargetPosition(pos);
            arm[i].setPower(1.0);
        }
    }

    public void swing(boolean up) {
        if (up) {
            for (int i = 0; i < 3; i++) {
                arm[i].setTargetPosition(5);
                extender.setTargetPosition(0);
            }
        } else {
            for (int i = 0; i < 3; i++) {
                if (extender.getCurrentPosition() < 100) { // placeholder
                    arm[i].setTargetPosition(400); // placeholder
                }
                arm[i].setTargetPosition((int) (Math.atan((armHeight - omniRadius) / (baseArmLength + ((extender.getCurrentPosition() * 2 * Math.PI * extensionRadius) / ticksPerRevolution))) * ticksPerRevolution / 360));
            }
        }
    }

    public void door(boolean open) {
        door.setPosition(open ? 0.75 : 0.25);
    }

    public void latch(Boolean lock) {
        hook.setPosition(lock ? closePos : openPos);
    }
}
