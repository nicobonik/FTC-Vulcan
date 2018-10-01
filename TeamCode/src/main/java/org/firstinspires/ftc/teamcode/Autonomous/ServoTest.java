package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeLift;

@Autonomous(name = "ServoTest", group="Auto")
public class ServoTest extends LinearOpMode {
    public void runOpMode() {
        Servo servo = hardwareMap.servo.get("servo");
        servo.setPosition(0.0);
        sleep(1000);
        servo.setPosition(0.0);
    }
}
