package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.ArrayMap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    private DcMotorEx[] motors = new DcMotorEx[4];
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;
    //private Navigation nav;
    public PID drivePID, turnPID;
    private PowerControl driveControl, turnControl;

    public Drivetrain(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight, BNO055IMU IMU) {
        motors[0] = frontLeft;
        motors[1] = frontRight;
        motors[2] = backLeft;
        motors[3] = backRight;
        imu = IMU;
        //nav = new Navigation(imu, cameraId);

        setM(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        setM(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        setZeroP(DcMotorEx.ZeroPowerBehavior.FLOAT);

        motors[0].setDirection(DcMotorEx.Direction.FORWARD);
        motors[1].setDirection(DcMotorEx.Direction.REVERSE);
        motors[2].setDirection(DcMotorEx.Direction.REVERSE);
        motors[3].setDirection(DcMotorEx.Direction.FORWARD); // should be 1, 3 reversed, idk what's up with electrical

        driveControl = new PowerControl() {
            public void setPower(double power) {
                speeds[0] = power;
                speeds[1] = power;
                speeds[2] = power;
                speeds[3] = power;
            }

            public double getPosition() {
                double sum = 0;
                for (int i = 0; i < 4; i++) {
                    sum += motors[i].getCurrentPosition() * (i < 2 ? 1 : -1);
                }
                return sum / 4 / ticksPerInch;
            }
        };

        drivePID = new PID(-0.03, -0.05, 0.05, 0.05, driveControl);
        drivePID.limitOutput(-1, 1);

        turnControl = new PowerControl() {
            public void setPower(double power) {
                speeds[0] += power;
                speeds[2] += power;
                speeds[1] += -power;
                speeds[3] += -power;
            }
            public double getPosition() {
                return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            }
        };
        turnPID = new PID(-0.01, -0.01, 0.05, 0.05, turnControl, -180, 180);
        turnPID.limitOutput(-1, 1);

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
        for(int i = 0; i < 4; i++) {
            speeds[i] = 0;
        }
        if(drivePIDActive) {
            for(int i = 0; i < 4; i++) {
                speeds[i] = 0;
            }
            drivePIDActive = drivePID.maintainOnce(driveTarget, driveMargin);
        }
        /*if(turnPIDActive) {
            turnPIDActive = turnPID.maintainOnce(turnTarget, turnMargin);
        }*/
        /*//experimental
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
        if(Math.abs(angle) > 10) {
            for (int i = 0; i < 4; i++) {
                speeds[i] = -Math.signum(angle) * 0.5;
            }
        }
        //*/
        double max = Math.max(Math.max(Math.max(Math.max(Math.abs(speeds[0]), Math.abs(speeds[1])), Math.abs(speeds[2])), Math.abs(speeds[3])), 1);
        for (int i = 0; i < 4; i++) {
            motors[i].setPower(tempPower * speeds[i] / max);
        }
        return telemetryPackets;
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
            double forward2 = tempPower * forward;
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

    public void mecanumDrive(double forward, double strafe, double turn, double multiplier) {
        if(!drivePIDActive) {
            double vd = Math.hypot(forward, strafe);
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
                speeds[i] = (multiplier * v[i]);
            }
            if (turn != 0) {
                turnPIDActive = false;
            } else {
                if (!turnPIDActive) {
                    turnTarget = (int) imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                }
                turnPIDActive = true;
            }
        }
    }

    public void driveTimed(double seconds, double forward, double turn) {
        drivePIDActive = false;
        ElapsedTime time = new ElapsedTime();
        time.reset();
        setM(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arcadeDrive(forward, turn);
        while(time.time() < seconds) {}
        stop();
    }

    public void driveEnc(double inches) {
        /*setM(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        setM(DcMotorEx.RunMode.RUN_USING_ENCODER);
        driveTarget = (int)(inches * ticksPerInch);
        drivePID.reset();
        drivePIDActive = true;*/
        setM(DcMotorEx.RunMode.RUN_TO_POSITION);
        for(int i = 0; i < 4; i++) {
            motors[i].setTargetPosition((i < 2 ? 1 : -1) * (int)(inches * ticksPerInch));
            motors[i].setPower(0.6);
        }
    }

    public void turn(int degrees) {
        setM(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turnTarget = (int)imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + degrees;
        turnPID.reset();
        turnPIDActive = true;
    }

    public void turnEnc(int degrees) {
        setM(DcMotorEx.RunMode.RUN_TO_POSITION);
        for(int i = 0; i < 4; i++) {
            motors[i].setTargetPosition((int)(degrees / 360 * 17 * Math.PI * ticksPerInch));
            motors[i].setPower(0.6);
        }
    }

    public void turn(double power) {
        setM(DcMotorEx.RunMode.RUN_USING_ENCODER);
        for(int i = 0; i < 2; i++) {
            motors[i].setPower(power);
        }
        for(int i = 2; i < 4; i++) {
            motors[i].setPower(-power);
        }
    }

    /*//todo: if inverse of heading is closer, turn to it and just drive backwards
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
    }*/

    public int heading() {
        return (int)imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public double[] getPosition() {
        double[] pos = new double[4];
        for (int i = 0; i < 4; i++) {
            pos[i] = motors[i].getCurrentPosition() / ticksPerInch;
        }
        return pos;
    }

    public String speeds() {
        StringBuilder speeds = new StringBuilder();
        for (DcMotorEx motor : motors) {
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
        setM(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setZeroP(DcMotorEx.ZeroPowerBehavior behavior) {
        for (int i = 0; i < 4; i++) {
            motors[i].setZeroPowerBehavior(behavior);
        }
    }

    private void setM(DcMotorEx.RunMode mode) {
        for (int motor = 0; motor < 4; motor++) {
            motors[motor].setMode(mode);
        }
    }

    /*private boolean busy() {
        for (DcMotorEx motor: motors) {
            if (motor.getMode() == DcMotorEx.RunMode.RUN_TO_POSITION && motor.isBusy()) {
                return true;
            }
        }
        return false;
    }*/

    public void whileBusy() {
        while(isBusy()) {}
    }

    public boolean isBusy() {
        for(int i = 0; i < 4; i++) {
            if(motors[i].isBusy()) {
                return true;
            }
        }
        return false;
        //return drivePIDActive || turnPIDActive;
    }
}