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

@Autonomous(name = "TestAutonomous", group="Auto")
public class Auto extends LinearOpMode{
    private Robot robot;
    private ElapsedTime Runtime = new ElapsedTime();
    private MineralVisionHough houghVision;
    private int[] goldPos = new int[3];
    public void runOpMode() {
        robot = new Robot(hardwareMap);
        houghVision = new MineralVisionHough();
        houghVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        houghVision.setShowCountours(false);
        houghVision.setTelem(telemetry);
        waitForStart();
        Runtime.reset();
        //unfold bot
        robot.arm.swing(true);
        robot.arm.whileBusy();
        //extend slides
        robot.arm.extendDist(10);
        robot.arm.whileBusy();
        //drive forward
        robot.drivetrain.resetOrientation(); //reset IMU because robot has landed
        robot.drivetrain.driveEnc(4);
        robot.drivetrain.whileBusy();
        //retract slides
        robot.arm.extend(false);
        //fold slides
        robot.arm.swing(false);
        //take video of sampling field, find most commonly detected gold position
        houghVision.enable();
        for(int i = 0; i < 10; i++) {
            goldPos[houghVision.getGoldPos()]++;
        }
        houghVision.disable();
        int max = 0;
        int argmax = 3;
        for(int i = 0; i < 2; i++) {
            if(goldPos[i] > max) {
                max = goldPos[i];
                argmax = i;
            }
        }
        //drive through gold position
        robot.drivetrain.turn(45 * (argmax - 1));
        robot.drivetrain.whileBusy();
        robot.drivetrain.driveEnc(6);
        robot.drivetrain.whileBusy();
        //drive to depot
        robot.drivetrain.turn(-90 * (argmax - 1));
        robot.drivetrain.whileBusy();
        robot.drivetrain.driveEnc(6);
        robot.drivetrain.whileBusy();
        robot.drivetrain.turn(45 * (argmax - 1));
        robot.drivetrain.whileBusy();
        robot.drivetrain.driveEnc(24);
        robot.drivetrain.whileBusy();
        //drop team marker
        robot.intake.intake(-1);
        sleep(500);
        robot.intake.stop();
        //park
        robot.drivetrain.turn(135);
        robot.drivetrain.whileBusy();
        robot.drivetrain.driveEnc(48);
        robot.drivetrain.whileBusy();
        robot.drivetrain.stop();
    }
}