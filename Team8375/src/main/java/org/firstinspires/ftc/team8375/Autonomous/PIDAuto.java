package org.firstinspires.ftc.team8375.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name = "PIDTest", group = "Test")
public class PIDAuto extends LinearOpMode {
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    private BNO055IMU imu;
    HardwareMap hardwareMap;
    public BNO055IMU.Parameters parameters;
    ElapsedTime time = new ElapsedTime();

    public int iteration = 0;
    public double error;
    public double previousError;
    public double integral;
    public double derivative;
    double sensorVal;
    double distanceTravelled;
    double errorSum;
    double output;
    boolean initOver;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while(opModeIsActive()){
            PIDMove_In(1, 1, 1, 1, 1, 1);
        }
    }

    public void PIDMove_In(
        double Kp,
        double Ki,
        double Kd,
        double inches,
        long iteration_time,
        double heading
        ) throws InterruptedException {
        do{
            distanceTravelled = frontLeft.getCurrentPosition()/537.6;
            sensorVal = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            error = heading - sensorVal;
            integral = integral + error * iteration_time;
            derivative = (error - previousError)/iteration_time;
            output = Kp*error + Ki*integral + Kd*derivative;

            //setPowers(output);
            previousError = error;
            //iteration time is in milliseconds
            wait(iteration_time);
            telemetry.addData("output", output);
        } while(distanceTravelled<inches);
    }
    public void Init() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
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

        frontLeft = hardwareMap.dcMotor.get("front_left");
        frontRight = hardwareMap.dcMotor.get("front_right");
        backLeft = hardwareMap.dcMotor.get("back_left");
        backRight = hardwareMap.dcMotor.get("back_right");
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        time.reset();
        initOver = true;
    }

    public void setPowers(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        backRight.setPower(-power);
    }
}
