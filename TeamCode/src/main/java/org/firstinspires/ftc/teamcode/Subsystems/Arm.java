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
    public volatile double swingPosition, extendPosition, swingPower, extendPower;
    private volatile boolean swingPIDActive, extendPIDActive;
    public DcMotor[] arm;
    private DcMotor extender;
    //private AnalogInput potentiometer;
    private PID extendPID, swingPID;
    private PowerControl extendControl, swingControl;
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

        arm[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extender.setDirection(DcMotor.Direction.REVERSE);

        extendControl = new PowerControl() {
            public void setPower(double power) {
                extender.setPower(power);
            }
            public double getPosition() {
                return extender.getCurrentPosition();
            }
        };

        extendPID = new PID(-1, 0, 0, 0, extendControl);

        swingControl = new PowerControl() {
            public void setPower(double power) {
                arm[0].setPower(power);
                arm[1].setPower(power);
            }

            public double getPosition() {
                return arm[1].getCurrentPosition() - arm[0].getCurrentPosition() / 2;
                //return (potentiometer.getVoltage() / maxVoltage) * 270;
            }
        };

        swingPID = new PID(-0.005, -0.003, 0.005, 0, swingControl);

        swingPID.limitOutput(-1.0, 1.0);

        swingPosition = 0;
        extendPosition = 0;
        telemetryPackets = new LinkedHashMap<>();
    }

    public LinkedHashMap<String, String> updateSubsystem() {
        if(swingPIDActive) {
            swingPIDActive = swingPID.maintainOnce(swingPosition, 2);
        } else {
            if((Math.abs(arm[1].getCurrentPosition()) > ticksPerRevolution / 4 && arm[1].getPower() > 0) || (Math.abs(arm[1].getCurrentPosition()) < 0 && arm[1].getPower() < 0)) {
                arm[0].setPower(0);
                arm[1].setPower(0);
                swingPower = 0;
            } else {
                boolean movingOut;
                int upperDist = (int)(ticksPerRevolution / 4) - arm[1].getCurrentPosition();
                int lowerDist = arm[1].getCurrentPosition();
                movingOut = (swingPower > 0 && upperDist < 500) || (swingPower < 0 && lowerDist < 500);
                int distance = Math.min(upperDist, lowerDist);
                double limit;
                if(movingOut) {
                    limit = Range.clip((distance / 500d), 0.1, 1.0);
                } else {
                    limit = Range.clip((distance / 500d), 0.3, 1.0);
                }
                double pow = Range.clip(swingPower, -limit, limit);

                arm[0].setPower(pow);
                arm[1].setPower(pow);
            }
        }
        if(extendPIDActive) {
            extendPIDActive = extendPID.maintainOnce(extendPosition, 2);
        } else {
            extender.setPower(extendPower);
        }
        return telemetryPackets;
    }

    public void swing(double speed) {
        if(speed != 0) {
            swingPIDActive = false;
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
        while(extendPIDActive || swingPIDActive) {}
    }

    public void stop() {
        arm[0].setPower(0);
        arm[1].setPower(0);
        extender.setPower(0);
    }
}
