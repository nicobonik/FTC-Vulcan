package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import java.util.LinkedHashMap;

public class Arm extends Subsystem {
    private final double armTicksPerRevolution = 5264 * 1.75;
    private final double extensionTicksPerRevolution = 103.6;
    private final double revsPerInch = 0.125 * 25.4;
    private final double maximumExtension = 200 / 25.4;
    //private final double maxVoltage;
    private volatile double swingPosition, extendPosition, swingPower, extendPower;
    private volatile boolean swingPIDActive, extendPIDActive;
    private DcMotorEx[] arm;
    private DcMotorEx extender;
    //private AnalogInput potentiometer;
    private PID extendPID, swingPID;
    private PowerControl extendControl, swingControl;
    private double[] extendPresets = new double[] {0, 2, 6, maximumExtension};
    private int extendCounter = 0;
    public Arm(DcMotorEx[] armMotors, DcMotorEx extend) {//, AnalogInput pot) {
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

        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extender.setDirection(DcMotor.Direction.REVERSE);

        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extendControl = new PowerControl() {
            public void setPower(double power) {
                extender.setPower(power);
            }
            public double getPosition() {
                return extender.getCurrentPosition() / extensionTicksPerRevolution / revsPerInch;
            }
        };

        extendPID = new PID(0.4, 0.05, 0.005, 0, extendControl);
        extendPID.limitOutput(-1.0, 1.0);
        swingControl = new PowerControl() {
            public void setPower(double power) {
                arm[0].setPower(power);
                arm[1].setPower(power);
            }

            public double getPosition() {
                return arm[1].getCurrentPosition() / armTicksPerRevolution * 360;
                //return (potentiometer.getVoltage() / maxVoltage) * 270;
            }
        };

        swingPID = new PID(0.02, 0.005, 0.00, 0, swingControl);

        swingPID.limitOutput(-1.0, 1.0);

        swingPosition = 0;
        extendPosition = 0;
        telemetryPackets = new LinkedHashMap<>();
    }

    public LinkedHashMap<String, String> updateSubsystem() {
        telemetryPackets.clear();
        if(arm[1].getCurrentPosition() < 0) {
            arm[1].setPower(0.1);
        }
        if(swingPIDActive) {
            swingPIDActive = swingPID.maintainOnce(swingPosition, 2);
        } else {
            if(Math.abs(arm[1].getCurrentPosition()) > armTicksPerRevolution / 4 + 75 && (arm[1].getPower() > 0 || swingPower > 0) || (Math.abs(arm[1].getCurrentPosition()) < 0 && (arm[1].getPower() < -75 || swingPower < 0))) {
                arm[0].setPower(0);
                arm[1].setPower(0);
                swingPower = 0;
            } else {
                boolean movingOut;
                int upperDist = (int)(armTicksPerRevolution / 4) - arm[1].getCurrentPosition();
                int lowerDist = arm[1].getCurrentPosition();
                movingOut = (swingPower > 0 && upperDist < 500) || (swingPower < 0 && lowerDist < 500);
                int distance = Math.min(upperDist, lowerDist);
                double limit;
                if(movingOut) {
                    limit = Range.clip((distance / 500d), 0.15, 1.0);
                } else {
                    limit = Range.clip((distance / 500d), 0.4, 1.0);
                }
                double pow = Range.clip(swingPower, -limit, limit);

                arm[0].setPower(pow);
                arm[1].setPower(pow);
            }
        }
        if(extendPIDActive) {
            extendPIDActive = extendPID.maintainOnce(extendPosition, 0.05);
        } else {
            extender.setPower(extendPower);
        }
        return telemetryPackets;
    }

    public void swing(double speed) {
        if(speed != 0) {
            swingPIDActive = false;
        } if (!swingPIDActive) {
            double power = swingPower;
            if (speed - swingPower >= 0.02) {
                power += 0.02;
            } else if (speed - swingPower <= -0.02) {
                power -= 0.02;
            }
            power = Math.round(power * 100) / 100.0;
            swingPower = Range.clip(power, -1.0, 1.0);
        }
    }

    public void swingAngle(double angle) {
        swingPIDActive = true;
        swingPosition = Math.max(Math.min(angle, 90), 0);
        swingPID.reset();
    }

    public void extend(double speed) {
        if(speed != 0) {
            extendPIDActive = false;
        }
        extendPower = speed;
    }

    public void extend(boolean out) {
        if(out) {
            extendCounter = Math.min(++extendCounter, 3);
            extendDist(extendPresets[extendCounter]);
        } else {
            extendCounter = Math.max(--extendCounter, 0);
            extendDist(extendPresets[extendCounter]);
        }
    }

    public void extendDist(double inches) {
        extendPIDActive = true;
        extendPosition = inches;
        extendPID.reset();
    }

    public void resetEnc() {
        arm[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void whileBusy() {
        while(isBusy()) {}
    }

    public boolean isBusy() {
        return extendPIDActive || swingPIDActive;
    }

    public void setZeroP(DcMotor.ZeroPowerBehavior behav) {
        arm[0].setZeroPowerBehavior(behav);
        arm[1].setZeroPowerBehavior(behav);
    }

    public void stop() {
        arm[0].setPower(0);
        arm[1].setPower(0);
        extender.setPower(0);
        arm[0].setMotorDisable();
        arm[1].setMotorDisable();
        extender.setMotorDisable();
    }
}
