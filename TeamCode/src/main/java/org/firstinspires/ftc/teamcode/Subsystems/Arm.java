package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import java.util.LinkedHashMap;
import java.util.Map;

public class Arm extends Subsystem {
    private final double ticksPerRevolution = 5264 * 1.75;
    private final double revsPerInch = 10; //placeholder
    private final double maximumExtension = 10000; //placeholder
    //private final double maxVoltage;
    private volatile double swingPosition, extendPosition, swingPower, extendPower;
    private volatile boolean swingPIDActive, extendPIDActive;
    public volatile boolean brake;
    private DcMotor[] arm;
    private DcMotor extender;
    //private AnalogInput potentiometer;
    private PID extendPID, swingPID;
    public Arm(DcMotor[] armMotors, DcMotor extend) {//, AnalogInput pot) {
        arm = armMotors;
        extender = extend;
        //potentiometer = pot;
        //maxVoltage = potentiometer.getMaxVoltage();

        arm[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm[0].setDirection(DcMotor.Direction.REVERSE);
        arm[1].setDirection(DcMotor.Direction.FORWARD);

        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extender.setDirection(DcMotor.Direction.REVERSE);

        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
                return arm[1].getCurrentPosition() - arm[0].getCurrentPosition() / 2;
                //return (potentiometer.getVoltage() / maxVoltage) * 270;
            }
        });

        swingPosition = 0;
        extendPosition = 0;
        telemetryPackets = new LinkedHashMap<>();
    }

    public LinkedHashMap<String, String> updateSubsystem() {
        if(swingPIDActive) {
            swingPID.maintainOnce(swingPosition, 2);
        } else {
            if((Math.abs(arm[1].getCurrentPosition()) > ticksPerRevolution / 4 && arm[1].getPower() > 0) || (Math.abs(arm[1].getCurrentPosition()) < 0 && arm[1].getPower() < 0)) {
                arm[0].setPower(0);
                arm[1].setPower(0);
                swingPower = 0;
            } else {
                int distance = (int)Math.min(arm[1].getCurrentPosition(), (ticksPerRevolution / 4) - arm[1].getCurrentPosition());
                double limit = Range.clip((distance / 400d), 0.075, 1.0);
                double pow = Range.clip(swingPower, -limit, limit);
                telemetryPackets.put("limit", Double.toString(limit));
                arm[0].setPower(pow);
                arm[1].setPower(pow);
            }
        }
        if(extendPIDActive) {
            extendPID.maintainOnce(extendPosition, 2);
        } else {
            extender.setPower(extendPower);
        }
        return telemetryPackets;
    }

    public void swing(double speed) {
        if(speed != 0) {
            swingPIDActive = false;
        } else {
            if(!swingPIDActive) {
                swingPosition = arm[1].getCurrentPosition();
            }
            swingPIDActive = true;
        }
        double power = (speed / 0.7) * (0.3 * Math.pow(speed, 6) + 0.4);
        swingPower = power;
    }

    public void swing(boolean up) {
        swingPIDActive = true;
        swingPosition = up ? ticksPerRevolution / 4 : 0;
    }

    public void extend(double speed) {
        extendPIDActive = false;
        extendPower = speed;
    }

    public void extend(boolean out) {
        extendPIDActive = true;
        extendPosition = out ? maximumExtension : 0;
    }

    public void extendDist(double inches) {
        extendPIDActive = true;
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
