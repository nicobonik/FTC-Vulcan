package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Drivetrain {
    private DcMotor fl, fr, bl, br;
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;

    public Drivetrain(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, BNO055IMU IMU) {
        fl = frontLeft;
        fr = frontRight;
        bl = backLeft;
        br = backRight;
        imu = IMU;

        fl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

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

    public void mecanumDrive(double forward, double strafe, double turn, double multiplier) {
        double vd = Math.hypot(forward, strafe);
        double theta = Math.atan2(forward, strafe) - (Math.PI / 4);
        double[] v = {
                vd * Math.sin(theta) + turn,
                vd * Math.cos(theta) - turn,
                vd * Math.cos(theta) + turn,
                vd * Math.sin(theta) - turn
        };
        for (int i = 0; i < 4; i++) {
            fl.setPower(multiplier * v[0]);
            bl.setPower(multiplier * v[1]);
            fr.setPower(multiplier * v[2]);
            br.setPower(multiplier * v[3]);
        }
    }

    public void driveForwards(int ticks) {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fl.setTargetPosition(ticks);
        bl.setTargetPosition(ticks);
        fr.setTargetPosition(-ticks);
        br.setTargetPosition(-ticks);
        while(fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy()) {}
    }

    public void turn(double degrees) {
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double error;
        do {
            error = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - degrees;
            fl.setPower(error * 0.025);
            bl.setPower(error * 0.025);
            fr.setPower(-error * 0.025);
            br.setPower(-error * 0.025);
        } while(Math.abs(error) > 3);
    }

    public void stop() {
    }
}