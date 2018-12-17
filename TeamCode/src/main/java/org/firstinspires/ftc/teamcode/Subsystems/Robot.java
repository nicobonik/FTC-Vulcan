package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public Drivetrain drivetrain;

    public Robot(HardwareMap hwMap) {
        drivetrain = new Drivetrain(
            hwMap.dcMotor.get("front_left"),
            hwMap.dcMotor.get("front_right"),
            hwMap.dcMotor.get("back_left"),
            hwMap.dcMotor.get("back_right"),
            hwMap.get(BNO055IMU.class, "imu")
        );
    }

    public void stop() {
        drivetrain.stop();
    }
}
