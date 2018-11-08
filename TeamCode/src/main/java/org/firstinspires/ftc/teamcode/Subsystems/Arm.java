package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Arm {
    private final double ticksPerRevolution = 1440;
    private final double revsPerInch = 10; //placeholder
    private final double maximumExtension = 10000; //placeholder
    private double swingPosition, extendPosition;
    private volatile boolean running, swingPIDActive, extendPIDActive;
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

        swingPosition = 0;
        extendPosition = 0;
    }

    public void init() {
        running = true;
        new Thread() {
            @Override
            public void run() {
                while(running) {
                    swingPID.maintainOnce(swingPosition, 2);
                    extendPID.maintainOnce(extendPosition, 2);
                }
            }
        }.start();
    }

    public void swing(double power) {
        swingPosition += power * 15;
        swingPosition = Math.max(swingPosition, 0);
    }

    public void swing(boolean up) {
        swingPosition = up ? ticksPerRevolution / 4 : 0;
    }

    public void extend(double power) {
        extender.setPower(power);
    }

    public void extend(boolean out) {
        extendPosition = out ? maximumExtension : 0;
    }

    public void extendDist(double inches) {
        extendPosition = extender.getCurrentPosition() + inches * revsPerInch * ticksPerRevolution;
    }

    public void whileBusy() {
        while(extendPID.busy || swingPID.busy) {}
    }
}