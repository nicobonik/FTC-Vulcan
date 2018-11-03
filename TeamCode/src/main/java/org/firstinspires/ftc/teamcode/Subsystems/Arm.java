package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Arm {
    private final double extensionRadius = 1;
    private final double ticksPerRevolution = 1440;
    private final double revsPerInch = 10;
    private final double maximumExtension = 10000;
    private int position = 0;
    private DcMotor[] arm;
    private DcMotor extender;
    private PID extendPID, swingPID;
    public Arm(DcMotor[] armMotors, DcMotor extend) {
        arm = armMotors;
        extender = extend;

        arm[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm[0].setDirection(DcMotor.Direction.FORWARD);
        arm[1].setDirection(DcMotor.Direction.REVERSE);

        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extendPID = new PID(1, 0, 0, 0, new PowerControl() {
            public void setPower(double power) {
                extender.setPower(power);
            }
            public double getPosition() {
                return extender.getCurrentPosition();
            }
        });

        swingPID = new PID(1, 0, 0, 0, new PowerControl() {
            public void setPower(double power) {
                arm[0].setPower(power);
                arm[1].setPower(power);
            }
            public double getPosition() {
                return (arm[0].getCurrentPosition() - arm[2].getCurrentPosition()) / 2;
            }
        });
    }

    public void swing(double power) {
        position += power * 15;
        int pos = Math.max(position, 0);
        swingPID.runToPosition(pos, 2);
    }

    public void swing(boolean up) {
        swingPID.runToPosition(up ? ticksPerRevolution / 4 : 0, 2);
    }

    public void extend(double power) {
        extender.setPower(power);
    }

    public void extend(boolean out) {
        extendPID.runToPosition(out ? maximumExtension : 0, 5);
    }

    public void extendDist(double inches) {
        extendPID.runToPosition((extender.getCurrentPosition() + inches * revsPerInch * ticksPerRevolution), 5);
    }

    public void whileBusy() {
        while(extendPID.busy || swingPID.busy) {}
    }
}