package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Drivetrain extends Subsystem {
    private static final double wheelCirc = Math.PI * 4;
    private static final double ticksPerInch = 537.6 / wheelCirc;
    private static final double turnMult = 0.8;
    private double[] speeds = new double[4];
    private double ly, lx, rx;
    private int driveTarget, driveMargin, turnTarget, turnMargin;
    private volatile boolean drivePIDActive, turnPIDActive, running;
    public static final double BASE_POWER = 0.9;
    public double tempPower = BASE_POWER;
    private DcMotor[] motors = new DcMotor[4];
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;
    private Orientation zeroOrientation;
    //private ModernRoboticsI2cGyro gyro;
    private Thread systemThread;
    public PID drivePID, turnPID;

    public Drivetrain(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, BNO055IMU IMU) {
        motors[0] = frontLeft;
        motors[1] = frontRight;
        motors[2] = backLeft;
        motors[3] = backRight;

        setM(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroP(DcMotor.ZeroPowerBehavior.FLOAT);

        motors[0].setDirection(DcMotor.Direction.FORWARD);
        motors[1].setDirection(DcMotor.Direction.REVERSE);
        motors[2].setDirection(DcMotor.Direction.FORWARD);
        motors[3].setDirection(DcMotor.Direction.REVERSE);

        drivePID = new PID(-0.4, -0.7, 0.7, 0, new PowerControl() {
            public void setPower(double power) {
                for (int i = 0; i < 4; i++) {
                    speeds[i] += power;
                }
            }

            public double getPosition() {
                double sum = 0;
                for (int i = 0; i < 4; i++) {
                    sum += motors[i].getCurrentPosition() * (i % 2 == 0 ? 1 : -1);
                }
                return sum / 4 / ticksPerInch;
            }
        });

        turnPID = new PID(-0.025, -0.035, 0.035, 0, new PowerControl() {
            public void setPower(double power) {
                for (int i = 0; i < 4; i++) {
                    speeds[i] += Range.clip(power / 180, -1.0, 1.0) * (i % 2 == 0 ? -1 : 1);
                }
            }
            public double getPosition() {
                return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            }
        }, -180, 180);

        setupIMU(IMU);

        tempPower = BASE_POWER;
        drivePIDActive = false;
        turnPIDActive = false;
        driveTarget = 0;
        driveMargin = 0;
        turnTarget = 0;
        turnMargin = 0;
        ly = 0;
        lx = 0;
        rx = 0;
    }

    public void updateSubsystem() {
        mecanumDrive(-ly, lx, rx, 0.8);
        /*if(drivePIDActive) {
            for(int i = 0; i < 4; i++) {
                speeds[i] = 0;
            }
            drivePIDActive = drivePID.maintainOnce(driveTarget, driveMargin);
        }
        if(turnPIDActive) {
            turnPIDActive = turnPID.maintainOnce(turnTarget, turnMargin);
        }*/
        double max = Math.max(Math.max(Math.max(Math.max(Math.abs(speeds[0]), Math.abs(speeds[1])), Math.abs(speeds[2])), Math.abs(speeds[3])), 1);
        for (int i = 0; i < 4; i++) {
            motors[i].setPower(speeds[i] / max);
        }
    }

    public void setGamepadState(double ly, double lx, double rx) {
        this.ly = ly;
        this.lx = lx;
        this.rx = rx;
    }

    /*public void setupGyro(ModernRoboticsI2cGyro gyr) {
        gyro = gyr;
        gyro.calibrate();
        while(gyro.isCalibrating()) {}
    }*/

    public void setupIMU(BNO055IMU adaIMU) {
        parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = adaIMU;

        imu.initialize(parameters);
        while (!imu.isGyroCalibrated()) {}
        resetOrientation();
    }

    /*public void resetOrientationGyro() {
        gyro.resetZAxisIntegrator();
    }*/

    public void resetOrientation()
    {
        zeroOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public void arcadeDrive(double forward, double turn) {
        if(!drivePIDActive) {
            double turn2 = turnMult * tempPower * (forward <= 0 ? 1 : -1) * turn;
            double forward2 = tempPower * (forward / 0.7) * (0.3 * Math.pow(forward, 6) + 0.4);
            double v[] = {forward2 + turn2, //fl
                    forward2 - turn2, //fr
                    forward2 + turn2, //bl
                    forward2 - turn2};  //br
            double max = Math.max(Math.abs(v[3]), Math.max(Math.abs(v[2]), Math.max(Math.abs(v[1]), Math.abs(v[0]))));
            if (max > tempPower) {
                for (int i = 0; i < 4; i++) {
                    motors[i].setPower((v[i] / max));
                }
            } else {
                for (int i = 0; i < 4; i++) {
                    motors[i].setPower(v[i]);
                }
            }
        }
        if(turn == 0) {
            turnPIDActive = true;
        } else {
            turnPIDActive = false;
            turnPID.reset();
        }
    }

    public void mecanumDrive(double forward, double strafe, double turn, double multiplier) {
        if(!drivePIDActive) {
            double vd = Math.hypot((forward / 0.7) * (0.3 * Math.pow(forward, 6) + 0.4), (strafe / 0.7) * (0.3 * Math.pow(strafe, 6) + 0.4));
            double theta = Math.atan2(forward, strafe) - (Math.PI / 4);
            double[] v = {
                    vd * Math.sin(theta) + turn,
                    vd * Math.cos(theta) - turn,
                    vd * Math.cos(theta) + turn,
                    vd * Math.sin(theta) - turn
            };
            if ((v[0] > 0 && v[1] > 0 && v[2] > 0 && v[3] > 0) || (v[0] < 0 && v[1] < 0 && v[2] < 0 && v[3] < 0)) {
                if (multiplier < 0.6) {
                    multiplier = 0.6;
                }
            }
            for (int i = 0; i < 4; i++) {
                //speeds[i] = (tempPower * multiplier * v[i]);
            }
            speeds = v;
        }
        if(turn != 0) {
            turnPIDActive = false;
        }
    }

    public void driveTimed(double seconds, double forward, double turn) {
        drivePIDActive = false;
        ElapsedTime time = new ElapsedTime();
        time.reset();
        setM(DcMotor.RunMode.RUN_USING_ENCODER);
        arcadeDrive(forward, turn);
        while(time.time() < seconds) {}
        stop();
    }

    public void driveEnc(double inches) {
        setM(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setM(DcMotor.RunMode.RUN_USING_ENCODER);
        driveTarget = (int)(inches * ticksPerInch);
        drivePID.reset();
        drivePIDActive = true;
    }

    /*public void turnEnc(int degrees) {
        double distance = botCirc * degrees / 360;
        setM(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setM(DcMotor.RunMode.RUN_TO_POSITION);
        for (int motor = 0; motor < 4; motor++) {
            motors[motor].setPower(turnMult * tempPower);
            motors[motor].setTargetPosition((int)(distance * ticksPerInch) * ((motor % 2 == 0) ? 1 : -1));
        }
        while (busy()) {}
        stop();
    }*/

    /*public void turnGyro(int degrees) {
        resetOrientationGyro();
        setM(DcMotor.RunMode.RUN_USING_ENCODER);
        turnTarget = degrees;
        turnPID.reset();
        turnPIDActive = true;
    }*/

    public void turn(int degrees) {
        resetOrientation();
        setM(DcMotor.RunMode.RUN_USING_ENCODER);
        turnTarget = degrees;
        turnPID.reset();
        turnPIDActive = true;
    }

    /*public void driveTo(double x, double y) throws InterruptedException {
        double distance = Math.sqrt((x - pX) * (x - pX) + (y - pY) * (y - pY));
        double angle = Math.atan((y - pY) / (x - pX));
        turnGyro((int)angle);
        driveEnc(distance);
    }*/

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
        //turnIMU(0);
        running = false;
        setM(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

    /*private boolean busy() {
        for (DcMotor motor: motors) {
            if (motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION && motor.isBusy()) {
                return true;
            }
        }
        return false;
    }*/

    public void whileBusy() {
        while(drivePIDActive || turnPIDActive) {}
    }
}