package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Drivetrain {
    //todo: have IMU/gyro stabilize driving
    private static final int wheelDiam = 4;
    private static final double wheelCirc = Math.PI * wheelDiam;
    private static final double botDiam = 17; // placeholder
    private static final double botCirc = Math.PI * botDiam;
    private static final int encTicks = 1440;
    private static final int ticksPerInch = (int)(encTicks / wheelCirc);
    private static final double turnMult = 1.0;
    public static final double BASE_POWER = 0.9;
    public double tempPower = BASE_POWER;
    private double pX, pY;
    private DcMotor motors[] = new DcMotor[4];
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;
    private Orientation lastOrientation;
    private double globalAngle;

    public Drivetrain(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        motors[0] = frontLeft;
        motors[1] = frontRight;
        motors[2] = backLeft;
        motors[3] = backRight;

        setM(DcMotor.RunMode.RUN_USING_ENCODER);
        for (int motor = 0; motor < 4; motor++) {
            motors[motor].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        motors[0].setDirection(DcMotor.Direction.REVERSE);
        motors[1].setDirection(DcMotor.Direction.FORWARD);
        motors[2].setDirection(DcMotor.Direction.REVERSE);
        motors[3].setDirection(DcMotor.Direction.FORWARD);
    }

    public void setupIMU(BNO055IMU revIMU) throws InterruptedException {
        parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = revIMU;

        imu.initialize(parameters);
        while (!imu.isGyroCalibrated())
        {
            Thread.sleep(50);
        }
        resetOrientation();
    }

    private void resetOrientation()
    {
        lastOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    public void arcadeDrive(double forward, double turn) {
        double turn2 = (forward <= 0 ? 1 : -1) * turn;
        double forward2 = (forward / 0.7) * (0.3 * Math.pow(forward, 6) + 0.4);
        double v[] = {tempPower * (forward2 + turnMult * turn2),
                tempPower * (forward2 - turnMult * turn2),
                tempPower * (forward2 + turnMult * turn2),
                tempPower * (forward2 - turnMult * turn2)};
        double max = Math.max(Math.abs(v[3]), Math.max(Math.abs(v[2]), Math.max(Math.abs(v[1]), Math.abs(v[0]))));
        if(max > tempPower) {
            for (int i = 0; i < 4; i++) {
                motors[i].setPower((v[i] / max));
            }
        } else {
            for (int i = 0; i < 4; i++) {
                motors[i].setPower(v[i]);
            }
        }
    }

    public void driveTimed(double seconds, double forward, double turn) {
        ElapsedTime time = new ElapsedTime();
        time.reset();
        setM(DcMotor.RunMode.RUN_USING_ENCODER);
        arcadeDrive(forward, turn);
        while(time.time() < seconds) {}
        stop();
    }

    public void driveEnc(double inches) {
        setM(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setM(DcMotor.RunMode.RUN_TO_POSITION); // alternatively, get current position and add to it - is faster
        for (int motor = 0; motor < 4; motor++) {
            motors[motor].setPower(tempPower);
            motors[motor].setTargetPosition(-(int)(inches * ticksPerInch));
        }
        while (busy()) {}
        stop();
    }

    public void driveStraight(double inches) {
        // todo
    }

    public void turnEnc(int degrees) {
        double distance = botCirc * degrees / 360;
        setM(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setM(DcMotor.RunMode.RUN_TO_POSITION);
        for (int motor = 0; motor < 4; motor++) {
            motors[motor].setPower(turnMult * tempPower);
            motors[motor].setTargetPosition((int)(distance * ticksPerInch) * ((motor % 2 == 0) ? 1 : -1));
        }
        while (busy()) {}
        stop();
    }

    public void turnGyro(int degrees) {
        // todo
    }

    public void turnIMU(int degrees) {
        resetOrientation();
        setM(DcMotor.RunMode.RUN_USING_ENCODER);
        int diff = degrees - (int)globalAngle;
        while (Math.abs(globalAngle - degrees) > 5) {
            globalAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            for (int motor = 0; motor < 4; motor++) {
                motors[motor].setPower(Range.clip((double)diff/30, 0.1, tempPower) * (motor % 2 == 0 ? -1 : 1));
            }
        }
        stop();
    }

    public void driveTo(double x, double y) {
        double distance = Math.sqrt((x - pX) * (x - pX) + (y - pY) * (y - pY));
        double angle = Math.atan((y - pY) / (x - pX));
        turnEnc((int)angle);
        driveEnc(distance);
    }

    public String speeds() {
        StringBuilder speeds = new StringBuilder();
        for (DcMotor motor : motors) {
            speeds.append(motor.getPower()).append(" ");
        }
        return speeds.toString();
    }

    public void speeds(double[] speeds) {
        for(int speed = 0; speed < 4; speed++) {
            motors[speed].setPower(speeds[speed]);
        }
    }

    public void stop() {
        arcadeDrive(0,0);
    }

    public void setZeroP(DcMotor.ZeroPowerBehavior behavior) {
        for (int i = 0; i < 4; i++) {
            motors[i].setZeroPowerBehavior(behavior);
        }
    }

    private void setM(DcMotor.RunMode mode) {
        for (int motor = 0; motor < 4; motor++) {
            motors[motor].setMode(mode);
        }
    }

    private boolean busy() {
        for (DcMotor motor: motors) {
            if (motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION && motor.isBusy()) {
                return true;
            }
        }
        return false;
    }
}