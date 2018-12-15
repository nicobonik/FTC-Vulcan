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

@Autonomous(name = "SilverAuto", group="Auto")
public class AutoSilver extends BaseAuto {
    public void runOpMode() {
        super.runOpMode();
        robot.drivetrain.turnEnc(50);
        while(robot.drivetrain.isBusy() && opModeIsActive()) {}
        runUntilGold();
        robot.arm.swingAngle(0);
        while(robot.arm.isBusy() && opModeIsActive()) {}
        robot.drivetrain.driveEnc(6);
        while(robot.drivetrain.isBusy() && opModeIsActive()) {}
        robot.drivetrain.driveEnc(-6);
        while(robot.drivetrain.isBusy() && opModeIsActive()) {}
        //robot.drivetrain.turn(-robot.drivetrain.heading() + 90);
        //while(robot.drivetrain.isBusy() && opModeIsActive()) {}
        robot.drivetrain.driveEnc(36 * Math.sqrt(2));
        while(robot.drivetrain.isBusy() && opModeIsActive()) {}
        robot.drivetrain.turn(45);
        while(robot.drivetrain.isBusy() && opModeIsActive()) {}
        robot.drivetrain.driveEnc(36);
        while(robot.drivetrain.isBusy() && opModeIsActive()) {}
        double startTime = getRuntime();
        robot.intake.intake(-0.8);
        while(opModeIsActive() && getRuntime() < startTime + 3) {}
        robot.intake.intake(0);
        robot.drivetrain.turn(180);
        while(robot.drivetrain.isBusy() && opModeIsActive()) {}
        robot.drivetrain.driveEnc(64);
        while(robot.drivetrain.isBusy() && opModeIsActive()) {}
        robot.arm.extendDist(4);
        while(robot.arm.isBusy() && opModeIsActive()) {}
        vis.disable();
        robot.stop();
    }
}