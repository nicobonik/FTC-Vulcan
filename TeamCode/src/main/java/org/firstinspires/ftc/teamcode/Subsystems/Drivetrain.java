package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static android.content.ContentValues.TAG;

public class Drivetrain {
    //todo: have IMU/gyro stabilize driving
    private static final int wheelDiam = 4;
    private static final double wheelCirc = Math.PI * wheelDiam;
    private static final double botDiam = 17; // placeholder
    private static final double botCirc = Math.PI * botDiam;
    private static final int encTicks = 1440;
    private static final int ticksPerInch = (int)(encTicks / wheelCirc);
    private static final double turnMult = 0.5;
    private double pX, pY;
    public static final double BASE_POWER = 0.9;
    public double tempPower = BASE_POWER;
    private DcMotor motors[] = new DcMotor[4];
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;
    private Orientation zeroOrientation;
    private ModernRoboticsI2cGyro gyro;
    public PID drivePID;
    //public PID turnPID;

    public Drivetrain(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        motors[0] = frontLeft;
        motors[1] = frontRight;
        motors[2] = backLeft;
        motors[3] = backRight;

        setM(DcMotor.RunMode.RUN_USING_ENCODER);
        for (int motor = 0; motor < 4; motor++) {
            motors[motor].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        motors[0].setDirection(DcMotor.Direction.FORWARD);
        motors[1].setDirection(DcMotor.Direction.REVERSE);
        motors[2].setDirection(DcMotor.Direction.FORWARD);
        motors[3].setDirection(DcMotor.Direction.REVERSE);
        Log.e(TAG, "Custom: motors initialized");
        PowerControl driveCtrl = new PowerControl() {
            public void setPower(double power) {
                for (int i = 0; i < 4; i++) {
                    motors[i].setPower(power);
                }
            }

            public double getPosition() {
                double sum = 0;
                for (int i = 0; i < 4; i++) {
                    sum += motors[i].getCurrentPosition() * (i % 2 == 0 ? 1 : -1);
                }
                return sum / 4;
            }
        };
        Log.e(TAG, "Custom: control initialized");

        drivePID = new PID(1, 0, 0, 0, driveCtrl);

        Log.e(TAG, "Custom: PID initialized");
        /*turnPID = new PID(0.4, 0.7, 0.7, 0, new PowerControl() {
            public void setPower(double power) {
                for (int motor = 0; motor < 4; motor++) {
                    motors[motor].setPower(Range.clip(power / 180, -1.0, 1.0) * (motor % 2 == 0 ? -1 : 1));
                }
            }
            public double getPosition() {
                return gyro.getHeading();
            }
        });*/
    }

    public void setupGyro(ModernRoboticsI2cGyro gyr) {
        gyro = gyr;
        gyro.calibrate();
        while(gyro.isCalibrating()) {}
    }

    public void setupIMU(BNO055IMU adaIMU) throws InterruptedException {
        parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = adaIMU;

        imu.initialize(parameters);
        while (!imu.isGyroCalibrated())
        {
            Thread.sleep(50);
        }
        resetOrientationIMU();
    }

    private void resetOrientationGyro() {
        gyro.resetZAxisIntegrator();
    }

    private void resetOrientationIMU()
    {
        zeroOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public void arcadeDrive(double forward, double turn) {
        double turn2 = turnMult * tempPower * (forward <= 0 ? 1 : -1) * turn;
        double forward2 = tempPower * (forward / 0.7) * (0.3 * Math.pow(forward, 6) + 0.4);
        double v[] = {forward2 + turn2, //fl
                forward2 - turn2, //fr
                forward2 + turn2, //bl
                forward2 - turn2};  //br
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

    public void mecanumDrive(double forward, double strafe, double turn, double multiplier) {
        double vd = Math.hypot(forward, strafe);
        double theta = Math.atan2(forward, strafe) - (Math.PI / 4);
        double[] v = {
                vd * Math.sin(theta) + turn,
                vd * Math.cos(theta) - turn,
                vd * Math.cos(theta) + turn,
                vd * Math.sin(theta) - turn
        };
        double max = Math.max(Math.max(Math.max(Math.max(Math.abs(v[0]), Math.abs(v[1])), Math.abs(v[2])), Math.abs(v[3])), 1);
        if ((v[0] > 0 && v[1] > 0 && v[2] > 0 && v[3] > 0) || (v[0] < 0 && v[1] < 0 && v[2] < 0 && v[3] < 0)) {
            if (multiplier < 0.6) {
                multiplier = 0.6;
            }
        }
        for(int i = 0; i < 4; i++) {
            motors[i].setPower(multiplier * v[i] / max);
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

    public void driveEnc(double inches) throws InterruptedException {
        setM(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setM(DcMotor.RunMode.RUN_USING_ENCODER);
        drivePID.runToPosition(inches * ticksPerInch, 2);
    }

    public void driveStraight(double inches) throws InterruptedException {
        resetOrientationGyro();
        driveEnc(inches);
        /*while(drivePID.busy) {
            if(Math.abs(gyro.getHeading()) > 1) {
                turnGyro(-gyro.getHeading());
            }
        }*/
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
        resetOrientationGyro();
        setM(DcMotor.RunMode.RUN_USING_ENCODER);
        //turnPID.runToPosition(degrees, 2);
        stop();
    }

    public void turnIMU(int degrees) throws InterruptedException {
        PID control = new PID(0.4, 0.7, 0.7, 0, new PowerControl() {
            public void setPower(double power) {
                for (int motor = 0; motor < 4; motor++) {
                    motors[motor].setPower(Range.clip(power / 180, -1.0, 1.0) * (motor % 2 == 0 ? -1 : 1));
                }
            }
            public double getPosition() {
                return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - zeroOrientation.firstAngle;
            }
        });
        resetOrientationIMU();
        setM(DcMotor.RunMode.RUN_USING_ENCODER);
        control.runToPosition(degrees, 5);
        stop();
    }

    public void driveTo(double x, double y) throws InterruptedException {
        double distance = Math.sqrt((x - pX) * (x - pX) + (y - pY) * (y - pY));
        double angle = Math.atan((y - pY) / (x - pX));
        turnGyro((int)angle);
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
        //driveEnc(0);
        //turnGyro(0);
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

    public void whileBusy() {
        //while(drivePID.busy || turnPID.busy) {}
    }
}