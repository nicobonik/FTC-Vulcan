package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class Arm {
    private final double ticksPerRevolution = 1440;
    private final double revsPerInch = 10; //placeholder
    private final double maximumExtension = 10000; //placeholder
    private double swingPosition, extendPosition;
    private volatile boolean running;
    private DcMotor[] arm;
    private DcMotor extender;
    private PID extendPID, swingPID;
    private Thread systemThread;
    public Arm(DcMotor[] armMotors, DcMotor extend) {
        arm = armMotors;
        extender = extend;

        arm[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm[0].setDirection(DcMotor.Direction.FORWARD);
        arm[1].setDirection(DcMotor.Direction.REVERSE);

        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extendPID = new PID(-1, 0, 0, 0, new PowerControl() {
            public void setPower(double power) {
                extender.setPower(power);
            }
            public double getPosition() {
                return extender.getCurrentPosition();
            }
        });

        swingPID = new PID(-1, 0, 0, 0, new PowerControl() {
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

        systemThread = new Thread() {
            @Override
            public void run() {
                while(running) {
                    swingPID.maintainOnce(swingPosition, 2);
                    extendPID.maintainOnce(extendPosition, 2);
                }
                swingPID.stop();
                extendPID.stop();
            }
        };
    }

    public void init() {
        running = true;
        systemThread.start();
    }

    public void swing(double speed) {
        swingPosition = Range.clip(speed * 15, 0, ticksPerRevolution / 4);
    }

    public void swing(boolean up) {
        swingPosition = up ? ticksPerRevolution / 4 : 0;
    }

    public void extend(double speed) {
        extendPosition = Range.clip(extendPosition + speed * 15, 0, maximumExtension);
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

    public void stop() {
        running = false;
    }
}