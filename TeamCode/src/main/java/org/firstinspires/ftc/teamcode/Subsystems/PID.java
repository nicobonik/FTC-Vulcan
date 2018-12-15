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
    private double lastPower = 0;
    private double lastTime = 0;
    private double bias = 0;
    private double minPos, maxPos, minPow = -1.0, maxPow = 1.0;
    private boolean cyclic = false, limit = true;
    private boolean firstLoop = true;
    private ElapsedTime timer = new ElapsedTime();
    private PowerControl control;
    public PID(double kp, double ki, double kd, double b, PowerControl ctrl) {
        Kp = kp;
        Ki = ki;
        Kd = kd;
        bias = b;
        control = ctrl;
        minPos = 0;
        maxPos = 0;
        cyclic = false;
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
        lastTime = timer.seconds();
        firstLoop = true;
    }

    private double getResponse(double currentPosition, double target) {
        double error = target - currentPosition;
        if(cyclic) {
            while(error > maxPos) {
                error -= maxPos - minPos;
            }
            while(error < minPos) {
                error += maxPos - minPos;
            }
        }
        //Proportional
        double response = (Kp * error);
        if(!firstLoop) {
            //Integral
            if(Math.abs(lastPower) < maxPow || !limit) {
                integral += (error * (timer.seconds() - lastTime));
                response += (Ki * integral);
            }
            //Derivative
            response += (Kd * (error - lastError) / (timer.seconds() - lastTime));
        }
        //Bias
        //response = Math.signum(response) * Math.max(bias, Math.abs(response));
        lastError = error;
        lastPower = response;
        lastTime = timer.seconds();
        firstLoop = false;
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
            double response = getResponse(control.getPosition(), target);
            if(limit) {
                control.setPower(Range.clip(response, minPow, maxPow));
            } else {
                control.setPower(response);
            }
            return true;
        }
        return false;
    }

    public void setCoefficients(double kp, double ki, double kd) {
        Kp = kp;
        Ki = ki;
        Kd = kd;
    }

    public double getPosition() {
        return control.getPosition();
    }

    public void stop() {
        control.setPower(0);
    }
}