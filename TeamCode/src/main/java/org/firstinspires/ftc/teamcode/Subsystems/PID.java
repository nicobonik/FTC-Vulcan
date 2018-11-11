package org.firstinspires.ftc.teamcode.Subsystems;

import android.content.ContentValues;
import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity.TAG;

public class PID {
    private double Kp, Ki, Kd;
    private double integral = 0;
    private double lastError = 0;
    private double lastTime = 0;
    private double bias = 0;
    private double minPos, maxPos, minPow = -1.0, maxPow = 1.0;
    private boolean cyclic = false, limit = true;
    private ElapsedTime timer = new ElapsedTime();
    private PowerControl control;
    public boolean busy = false;
    public PID(double kp, double ki, double kd, double b, PowerControl ctrl) {
        Kp = kp;
        Ki = ki;
        Kd = kd;
        bias = b;
        control = ctrl;
    }

    public PID(double kp, double ki, double kd, double b, PowerControl ctrl, double min, double max) {
        Kp = kp;
        Ki = ki;
        Kd = kd;
        bias = b;
        control = ctrl;
        minPos = min;
        maxPos = max;
        cyclic = true;
    }

    public void limitOutput(double min, double max) {
        limit = true;
        minPow = min;
        maxPow = max;
    }

    public void reset() {
        integral = 0;
        lastError = 0;
        timer.reset();
        lastTime = timer.time();
    }

    private double getResponse(double currentValue, double target) {
        double error = currentValue - target;
        if(cyclic) {
            while(error > maxPos - minPos) {
                error -= maxPos - minPos;
            }
            while(error < minPos - maxPos) {
                error += maxPos - minPos;
            }
        }
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

    public void runToPosition(double target, double margin) {
        reset();
        while (Math.abs(control.getPosition() - target) > margin) {
            if(limit) {
                control.setPower(Range.clip(getResponse(control.getPosition(), target), minPow, maxPow));
            } else {
                control.setPower(getResponse(control.getPosition(), target));
            }
        }
        control.setPower(0);
    }

    public boolean maintainOnce(double target, double margin) {
        if (Math.abs(control.getPosition() - target) > margin) {
            if(limit) {
                control.setPower(Range.clip(getResponse(control.getPosition(), target), minPow, maxPow));
            } else {
                control.setPower(getResponse(control.getPosition(), target));
            }
            return true;
        }
        return false;
    }

    public void stop() {
        control.setPower(0);
    }
}