package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import java.util.LinkedHashMap;

public class Arm extends Subsystem {
    private final double ticksPerRevolution = 5264 * 1.75;
    private final double revsPerInch = 10; //placeholder
    private final double maximumExtension = -10000; //placeholder
    //private final double maxVoltage;
    public volatile double swingTarget, extendTarget, swingPower, extendPower;
    private volatile boolean swingPIDActive, extendPIDActive;
    public DcMotorEx[] arm;
    private DcMotorEx extender;
    //private AnalogInput potentiometer;
    private PID extendPID, swingPID;
    private PowerControl extendControl, swingControl;
    private double[] extendPositions = {0, 8, 10, 15};
    private int currentExtendPos;
    public Arm(DcMotorEx[] armMotors, DcMotorEx extend) {//, AnalogInput pot) {
        arm = armMotors;
        extender = extend;
        //potentiometer = pot;
        //maxVoltage = potentiometer.getMaxVoltage();

        arm[0].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm[1].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        arm[0].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm[1].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        arm[0].setDirection(DcMotorEx.Direction.REVERSE);
        arm[1].setDirection(DcMotorEx.Direction.FORWARD);

        arm[0].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm[1].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        extender.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        extender.setDirection(DcMotorEx.Direction.REVERSE);

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

        swingTarget = 0;
        extendTarget = 0;
        currentExtendPos = 0;
        telemetryPackets = new LinkedHashMap<>();
    }

    public LinkedHashMap<String, String> updateSubsystem() {
        if(swingPIDActive) {
            swingPIDActive = swingPID.maintainOnce(swingTarget, 2);
        } else {
            if((Math.abs(arm[1].getCurrentPosition()) > ticksPerRevolution / 4 && arm[1].getPower() > 0) || (Math.abs(arm[1].getCurrentPosition()) < 0 && arm[1].getPower() < 0)) {
                arm[0].setPower(0);
                arm[1].setPower(0);
                swingPower = 0;
            } else {
                int distance = (int)Math.min(arm[1].getCurrentPosition(), (ticksPerRevolution / 4) - arm[1].getCurrentPosition());
                double limit = Range.clip((distance / 400d), 0.15, 1.0);
                double pow = Range.clip(swingPower, -limit, limit);

                arm[0].setPower(pow);
                arm[1].setPower(pow);
            }
        }
        if(extendPIDActive) {
            extendPIDActive = extendPID.maintainOnce(extendTarget, 2);
        } else {
            if(extender.getCurrentPosition() < maximumExtension) {
                extender.setPower(Math.max(0, extendPower));
            } else if (extender.getCurrentPosition() > 0) {
                extender.setPower(Math.min(0, extendPower));
            } else {
                extender.setPower(extendPower);
            }
        }
        return telemetryPackets;
    }

    public void swing(double speed) {
        if(speed != 0) {
            swingPIDActive = false;
        }
        swingPower = speed;
    }

    public void swing(boolean up) {
        swingPIDActive = true;
        swingTarget = up ? ticksPerRevolution / 4 : 0;
    }

    public void extend(double speed) {
        extendPIDActive = false;
        extendPower = speed;
    }

    public void extend(boolean out) {
        if(out) {
            currentExtendPos = Math.min(++currentExtendPos, 3);
        } else {
            currentExtendPos = Math.max(--currentExtendPos, 0);
        }
        extendDist(extendPositions[currentExtendPos]);
        extendPIDActive = true;
    }

    public void extendDist(double inches) {
        extendPIDActive = true;
        extendTarget = inches * revsPerInch * ticksPerRevolution;
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
