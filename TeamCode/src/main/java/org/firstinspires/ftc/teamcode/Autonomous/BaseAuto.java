package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.MineralVisionHough;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@Autonomous(name = "TestAuto", group="Auto")
public class BaseAuto extends LinearOpMode {
    private Robot robot;
    public void runOpMode() {
        /*DcMotor fl = hardwareMap.dcMotor.get("front_left");
        DcMotor fr = hardwareMap.dcMotor.get("front_right");
        DcMotor bl = hardwareMap.dcMotor.get("back_left");
        DcMotor br = hardwareMap.dcMotor.get("back_right");*/
        robot = new Robot(hardwareMap, telemetry);
        robot.init();
        waitForStart();
        robot.drivetrain.driveEnc(6);
        while(robot.drivetrain.isBusy() && opModeIsActive()) {
            telemetry.addData("target", robot.drivetrain.driveTarget);
            telemetry.addData("speed", robot.drivetrain.speeds());
            telemetry.addData("position", robot.drivetrain.getPosition());
            telemetry.addData("error", robot.drivetrain.getPosition() - robot.drivetrain.driveTarget);
            telemetry.update();
        }
        telemetry.addData("active", false);
        telemetry.update();
        //robot.drivetrain.turn(90);
        //robot.drivetrain.turn(-90);
        robot.drivetrain.driveEnc(-6);
        while(robot.drivetrain.isBusy() && opModeIsActive()) {
            telemetry.addData("active", false);
            telemetry.addData("target", robot.drivetrain.driveTarget);
            telemetry.addData("speed", robot.drivetrain.speeds());
            telemetry.update();
        }
        robot.stop();
    }
}