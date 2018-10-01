package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Subsystems.powerControl;

public class PID {
    private double Kp, Ki, Kd;
    private double target = 0;
    private double integral = 0;
    private double lastError = 0;
    private double lastTime = 0;
    private double bias = 0;
    private ElapsedTime timer = new ElapsedTime();
    private powerControl control;
    public PID(double kp, double ki, double kd, double b, powerControl ctrl) {
        Kp = kp;
        Ki = ki;
        Kd = kd;
        bias = b;
        control = ctrl;
    }

    public double getResponse(double currentValue) {
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

    public void runToPosition(double position, double margin) {
        target = position;
        timer.reset();
        lastTime = timer.time();
        while (Math.abs(position - target) > margin) {
            control.setPower(getResponse(control.getPosition()));
        }
    }
}