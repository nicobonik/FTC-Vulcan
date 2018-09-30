package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeLift;

@Autonomous(name = "TestAutonomous", group="Auto")
public class Auto extends LinearOpMode{
    private Drivetrain drivetrain;
    private IntakeLift inlift;
    private ElapsedTime Runtime = new ElapsedTime();
    public void runOpMode() {
        drivetrain = new Drivetrain(
                hardwareMap.dcMotor.get("front_left"),
                hardwareMap.dcMotor.get("front_right"),
                hardwareMap.dcMotor.get("back_left"),
                hardwareMap.dcMotor.get("back_right")
        );
        inlift = new IntakeLift(new DcMotor[] {hardwareMap.dcMotor.get("arm1"), hardwareMap.dcMotor.get("arm2"), hardwareMap.dcMotor.get("arm3")},
                hardwareMap.dcMotor.get("intake"),
                hardwareMap.servo.get("door")
                );
        /*inlift = new IntakeLift(hardwareMap.dcMotor.get("arm"),
                hardwareMap.dcMotor.get("intake"),
                hardwareMap.servo.get("hook"));*/
        /*try {
            drivetrain.setupIMU(hardwareMap.get(BNO055IMU.class, "imu"));
        } catch (InterruptedException e) {
            e.printStackTrace();
            requestOpModeStop();
        }*/
        waitForStart();
        Runtime.reset();

        drivetrain.stop();
    }
}