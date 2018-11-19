package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

public class Arm extends Subsystem {
    private final double ticksPerRevolution = 1440;
    private final double revsPerInch = 10; //placeholder
    private final double maximumExtension = 10000; //placeholder
    private final double maxVoltage;
    private double swingPosition, extendPosition;
    private volatile boolean running;
    private DcMotor[] arm;
    private DcMotor extender;
    private AnalogInput potentiometer;
    private PID extendPID, swingPID;
    private Thread systemThread;
    public Arm(DcMotor[] armMotors, DcMotor extend, AnalogInput pot) {
        arm = armMotors;
        extender = extend;
        potentiometer = pot;
        maxVoltage = potentiometer.getMaxVoltage();

        arm[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm[0].setDirection(DcMotor.Direction.REVERSE);
        arm[1].setDirection(DcMotor.Direction.FORWARD);

        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extender.setDirection(DcMotor.Direction.REVERSE);

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
                return arm[0].getCurrentPosition() - arm[1].getCurrentPosition() / 2;
                //return (potentiometer.getVoltage() / maxVoltage) * 270;
            }
        });

        swingPosition = 0;
        extendPosition = 0;
    }

    public void updateSubsystem() {
        swingPID.maintainOnce(swingPosition, 2);
        extendPID.maintainOnce(extendPosition, 2);
    }

    public void swing(double speed) {
        swingPosition = Range.clip(speed * 15, 0, 90);
    }

    public void swing(boolean up) {
        swingPosition = up ? 90 : 0;
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
        arm[0].setPower(0);
        arm[1].setPower(0);
        extender.setPower(0);
    }
}