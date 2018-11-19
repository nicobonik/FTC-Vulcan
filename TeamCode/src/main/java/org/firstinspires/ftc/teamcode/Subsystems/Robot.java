package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public Drivetrain drivetrain;
    public Intake intake;
    public Arm arm;

    public Robot(HardwareMap hardwareMap) {
        drivetrain = new Drivetrain(
                hardwareMap.dcMotor.get("front_left"),
                hardwareMap.dcMotor.get("front_right"),
                hardwareMap.dcMotor.get("back_left"),
                hardwareMap.dcMotor.get("back_right"),
                hardwareMap.get(BNO055IMU.class, "imu")
        );
        arm = new Arm(
                new DcMotor[] {hardwareMap.dcMotor.get("arm_left"), hardwareMap.dcMotor.get("arm_right")},
                hardwareMap.dcMotor.get("extender"),
                hardwareMap.analogInput.get("potent")
        );
        intake = new Intake(
                hardwareMap.dcMotor.get("intake"),
                hardwareMap.servo.get("door")
        );
    }

    public void stop() {
        drivetrain.stop();
        arm.stop();
        intake.stop();
    }
}
