package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Arm {
    private final double baseArmLength = 16;
    private final double armAngleOffset = 5;
    private final double armHeight = 6;
    private final double omniRadius = 2;
    private final double extensionRadius = 1;
    private final double ticksPerRevolution = 1440;
    private int position = 0;
    private DcMotor[] arm;
    private DcMotor extender;
    public Arm(DcMotor[] arm, DcMotor extender) {
        this.arm = arm;
        this.extender = extender;

        arm[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm[0].setDirection(DcMotor.Direction.FORWARD);
        arm[1].setDirection(DcMotor.Direction.REVERSE);
    }

    public void swing(double power) {
        PowerControl control = new PowerControl() {
            public void setPower(double power) {
                arm[0].setPower(power);
                arm[1].setPower(power);
            }
            public double getPosition() {
                return arm[0].getCurrentPosition();
            }
        };

        PID pid = new PID(1, 0, 0, 0, control);
        int pos = Math.min(position + (int)((power / 0.7) * (0.3 * Math.pow(power, 6) + 0.4) * 10), (int)(Math.atan((armHeight - omniRadius) / (baseArmLength + ((extender.getCurrentPosition() * 2 * Math.PI * extensionRadius) / ticksPerRevolution))) * ticksPerRevolution / 360));
        pos = Math.max(pos, 0);
        pid.runToPosition(pos, 2);
    }

    public void swing(boolean up) {
        if (up) {
            for (int i = 0; i < 3; i++) {
                arm[i].setTargetPosition(5);
                extender.setTargetPosition(0);
                arm[i].setPower(0.7);
            }
        } else {
            for (int i = 0; i < 3; i++) {
                if (extender.getCurrentPosition() < 100) { // placeholder
                    arm[i].setTargetPosition(400); // placeholder
                }
                arm[i].setTargetPosition((int) (Math.atan((armHeight - omniRadius) / (baseArmLength + ((extender.getCurrentPosition() * 2 * Math.PI * extensionRadius) / ticksPerRevolution))) * ticksPerRevolution / 360));
                arm[i].setPower(0.5);
            }
        }
        extender.setPower(0.9);
    }
}

class Runner implements Runnable {
    PID pid;
    public Runner(PID pid) {
        this.pid = pid;
    }

    public void setPosition(double position) {

    }

    public void run() {

    }
}