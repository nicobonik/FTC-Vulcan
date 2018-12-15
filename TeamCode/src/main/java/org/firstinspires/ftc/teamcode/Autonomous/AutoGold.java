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
import org.firstinspires.ftc.teamcode.Subsystems.MineralVisionContour;
import org.firstinspires.ftc.teamcode.Subsystems.MineralVisionHough;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@Autonomous(name = "GoldAuto", group="Auto")
public class AutoGold extends BaseAuto {
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        robot.drivetrain.setupIMU();
        robot.init();
        waitForStart();
        telemetry.addData("started", "true");
        telemetry.update();
        robot.drivetrain.driveEnc(36 * Math.sqrt(2));
        while(robot.drivetrain.isBusy() && opModeIsActive()) {}
        double startTime = getRuntime();
        robot.intake.intake(-0.8);
        while(opModeIsActive() && getRuntime() < startTime + 3) {}
        robot.intake.intake(0);
        robot.drivetrain.turn(135);
        while(robot.drivetrain.isBusy() && opModeIsActive()) {}
        robot.drivetrain.driveEnc(72);
        robot.stop();
    }
}