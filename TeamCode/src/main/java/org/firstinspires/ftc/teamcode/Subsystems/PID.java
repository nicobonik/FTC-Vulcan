package org.firstinspires.ftc.teamcode.Subsystems;

import android.content.ContentValues;
import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity.TAG;

public class PID {
    private double Kp, Ki, Kd;
    private double position = 0;
    private double margin = 0;
    private double integral = 0;
    private double lastError = 0;
    private double lastTime = 0;
    private double bias;
    private Thread runner;
    private ElapsedTime timer = new ElapsedTime();
    private PowerControl control;
    public boolean busy = false;
    public PID(double kp, double ki, double kd, double b, PowerControl ctrl) {
        Kp = kp;
        Ki = ki;
        Kd = kd;
        bias = b;
        control = ctrl;
        Log.e(TAG, "Custom: PID coefficients initialized");
        runner = new Thread() {
            public void run() {
                while (!this.isInterrupted() && Math.abs(control.getPosition() - position) > margin) {
                    control.setPower(getResponse(control.getPosition(), position));
                }
                control.setPower(0);
            }
        };
        Log.e(TAG, "Custom: Thread initialized");
    }

    private double getResponse(double currentValue, double target) {
        double error = currentValue - target;

        //Proportional
        double response = (Kp * error);
        //Integral
        integral += (error * (timer.time() - lastTime));
        response += (Ki * integral);
        //Derivative
        response += (Kd * (error - lastError) / (timer.time() - lastTime));
        //Bias
        response += bias;

        lastError = error;
        return response;
    }

    public void runToPosition(double position, double margin) throws InterruptedException {
        timer.reset();
        lastTime = timer.time();
        if(runner != null) {
            runner.interrupt();
            runner.join();
        }
        this.position = position;
        this.margin = margin;
        runner.start();
    }

    public void maintainOnce(double position, double margin) {
        timer.reset();
        lastTime = timer.time();
        if (Math.abs(control.getPosition() - position) > margin) {
            control.setPower(getResponse(control.getPosition(), position));
        }
    }
}