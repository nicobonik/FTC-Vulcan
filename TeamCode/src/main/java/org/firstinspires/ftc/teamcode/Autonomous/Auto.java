package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeLift;

@Autonomous(name = "TestAutonomous", group="Auto")
public class Auto extends LinearOpMode{
    private Drivetrain drivetrain;
    private IntakeLift inLift;
    private ElapsedTime Runtime = new ElapsedTime();
    public void runOpMode() {
        drivetrain = new Drivetrain(
                hardwareMap.dcMotor.get("front_left"),
                hardwareMap.dcMotor.get("front_right"),
                hardwareMap.dcMotor.get("back_left"),
                hardwareMap.dcMotor.get("back_right")
        );
        inLift = new IntakeLift(hardwareMap.dcMotor.get("arm"),
                hardwareMap.dcMotor.get("intake"),
                hardwareMap.servo.get("hook"));
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