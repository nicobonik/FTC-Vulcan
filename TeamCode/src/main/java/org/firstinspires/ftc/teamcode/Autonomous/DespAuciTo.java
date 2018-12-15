package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@Autonomous(name = "despAUciTO", group = "ThisIsWhatARTWouldDo")
public class DespAuciTo extends LinearOpMode {
    Robot robot;
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        robot.init();
        waitForStart();
        robot.arm.extendDist(7.5);
        while(opModeIsActive()) {
        }
        robot.stop();
    }
}
