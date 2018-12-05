package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.ArrayMap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Map;

public class Drivetrain extends Subsystem {
    private static final double wheelCirc = Math.PI * 4;
    private static final double ticksPerInch = 537.6 / wheelCirc;
    private static final double turnMult = 0.8;
    private double[] speeds = new double[4];
    private double ly, lx, rx;
    private int driveTarget, driveMargin, turnTarget, turnMargin;
    private volatile boolean drivePIDActive, turnPIDActive;
    private boolean fieldCentric;
    public static final double BASE_POWER = 0.9;
    public double tempPower;
    public double rearMultiplier = 1.0;
    private DcMotor[] motors = new DcMotor[4];
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;
    private Navigation nav;
    private Thread systemThread;
    public PID drivePID, turnPID;

    public Drivetrain(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, BNO055IMU IMU, int cameraId) {
        motors[0] = frontLeft;
        motors[1] = frontRight;
        motors[2] = backLeft;
        motors[3] = backRight;
        imu = IMU;
        nav = new Navigation(imu, cameraId);

        setM(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setM(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroP(DcMotor.ZeroPowerBehavior.FLOAT);

        motors[0].setDirection(DcMotor.Direction.FORWARD);
        motors[1].setDirection(DcMotor.Direction.REVERSE);
        motors[2].setDirection(DcMotor.Direction.REVERSE);
        motors[3].setDirection(DcMotor.Direction.FORWARD);

        drivePID = new PID(-0.4, -0.7, 0.7, 0.05, new PowerControl() {
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

        turnPID = new PID(-0.067, -0.035, 0.035, 0.05, new PowerControl() {
            public void setPower(double power) {
                for (int i = 0; i < 4; i++) {
                    speeds[i] += Range.clip(power, -1.0, 1.0) * (i % 2 == 0 ? -1 : 1);
                }
            }
            public double getPosition() {
                return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            }
        }, -180, 180);

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
        telemetryPackets = new LinkedHashMap<>();
    }

    public LinkedHashMap<String, String> updateSubsystem() {
        mecanumDrive(-ly, lx, rx, 0.8);
        if(drivePIDActive) {
            for(int i = 0; i < 4; i++) {
                speeds[i] = 0;
            }
            drivePIDActive = drivePID.maintainOnce(driveTarget, driveMargin);
        }
        if(turnPIDActive) {
            turnPIDActive = turnPID.maintainOnce(turnTarget, turnMargin);
        }
        //experimental
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
        if(Math.abs(angle) > 10) {
            for (int i = 0; i < 4; i++) {
                speeds[i] = -Math.signum(angle) * 0.5;
            }
        }
        //
        double max = Math.max(Math.max(Math.max(Math.max(Math.abs(speeds[0]), Math.abs(speeds[1])), Math.abs(speeds[2])), Math.abs(speeds[3])), 1);
        for (int i = 0; i < 4; i++) {
            motors[i].setPower(tempPower * speeds[i] / max);
        }
        return telemetryPackets;
    }

    public void setGamepadState(double ly, double lx, double rx) {
        this.ly = ly;
        this.lx = lx;
        this.rx = rx;
    }

    public void slow(double slow) {
        tempPower = Math.min(slow, BASE_POWER);
    }

    /*public void setupGyro(ModernRoboticsI2cGyro gyr) {
        gyro = gyr;
        gyro.calibrate();
        while(gyro.isCalibrating()) {}
    }*/

    public void setupIMU() {
        parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        if(imu.initialize(parameters)) {
            while (!imu.isGyroCalibrated()) {}
        } else {
            imu.initialize(parameters);
        }
    }

    public void arcadeDrive(double forward, double turn) {
        if(!drivePIDActive) {
            double turn2 = turnMult * tempPower * (forward <= 0 ? 1 : -1) * turn;
            double forward2 = tempPower * (forward / 0.7) * (0.3 * Math.pow(forward, 6) + 0.4);
            double v[] = {forward2 + turn2, //fl
                    forward2 - turn2, //fr
                    forward2 + turn2, //bl
                    forward2 - turn2};  //br
            double max = Math.max(Math.max(Math.abs(v[3]), Math.max(Math.abs(v[2]), Math.max(Math.abs(v[1]), Math.abs(v[0])))), 1);
            if (max > tempPower) {
                for (int i = 0; i < 4; i++) {
                    speeds[i] = v[i] / max;
                }
            } else {
                for (int i = 0; i < 4; i++) {
                    speeds[i] = (tempPower * v[i]);
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

    private void mecanumDrive(double forward, double strafe, double turn, double multiplier) {
        if(!drivePIDActive) {
            double vd = Math.hypot((forward / 0.7) * (0.3 * Math.pow(forward, 6) + 0.4), (strafe / 0.7) * (0.3 * Math.pow(strafe, 6) + 0.4));
            double theta = Math.atan2(forward, strafe) - (Math.PI / 4);
            if(fieldCentric) {
                theta -= heading();
            }
            //todo: add field centric turning
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
                speeds[i] = (multiplier * v[i]);
            }
        }
        if(turn != 0) {
            turnPIDActive = false;
        } else {
            if(!turnPIDActive) {
                turnTarget = (int)imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            }
            turnPIDActive = true;
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

    public void turn(int degrees) {
        setM(DcMotor.RunMode.RUN_USING_ENCODER);
        turnTarget = (int)imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + degrees;
        turnPID.reset();
        turnPIDActive = true;
    }

    //todo: if inverse of heading is closer, turn to it and just drive backwards
    public void driveTo(double x, double y) {
        double distance = Math.hypot((x - nav.x), (y - nav.y));
        double angle = Math.atan2(y - nav.y, x - nav.x);
        turn((int)(angle - nav.hdg));
        driveEnc(distance);
    }

    public void driveTo(int node) {
        ArrayList<Integer> path = nav.aStar(nav.closestNode(), node);
        while(path.size() > 0) {
            double[] nextNodePos = nav.nodePos(path.get(path.size() - 1));
            path.remove(path.size() - 1);
            driveTo(nextNodePos[0], nextNodePos[1]);
        }
    }

    public int heading() {
        return (int)nav.hdg;
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
        drivePIDActive = false;
        turnPIDActive = false;
        speeds(new double[] {0, 0, 0, 0});
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