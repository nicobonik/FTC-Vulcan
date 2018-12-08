package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "ServoTest", group="Testing")
public class ServoTest extends LinearOpMode {
    public void runOpMode() {
        CRServo servo1 = hardwareMap.crservo.get("servo1");
        CRServo servo2 = hardwareMap.crservo.get("servo2");
        servo1.setDirection(CRServo.Direction.FORWARD);
        servo2.setDirection(CRServo.Direction.REVERSE);
        waitForStart();
        servo1.setPower(1.0);
        servo2.setPower(1.0);
        while(opModeIsActive()) {}
        servo1.setPower(0);
        servo2.setPower(0);
    }
}
