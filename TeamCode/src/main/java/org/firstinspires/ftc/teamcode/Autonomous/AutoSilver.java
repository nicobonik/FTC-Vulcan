package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.Subsystems.MineralVisionContour;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@Autonomous(name = "TestAutonomous", group="Auto")
public class AutoSilver extends LinearOpMode {
    private Robot robot;
    private ElapsedTime Runtime = new ElapsedTime();
    private MineralVisionContour contourVision;
    private int[] goldPos = new int[3];
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        contourVision = new MineralVisionContour();
        contourVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        contourVision.setShowCountours(false);
        //start on ground, init IMU, then pull up before match starts
        robot.drivetrain.setupIMU(); //wait to start IMU until robot has landed
        double startTime = getRuntime();
        robot.arm.extendDist(-6); // placeholder
        robot.arm.swing(false);
        robot.arm.whileBusy();
        waitForStart();
        Runtime.reset();
        //unfold bot
        robot.arm.swing(true);
        robot.arm.whileBusy();
        //extend slides
        startTime = getRuntime();
        robot.arm.extend(1.0);
        while(getRuntime() < startTime + 0.5) {}
        robot.arm.extend(0);
        robot.arm.whileBusy();
        //drive forward
        robot.drivetrain.driveEnc(4);
        robot.drivetrain.whileBusy();
        //retract slides
        startTime = getRuntime();
        robot.arm.extend(-1.0);
        while(getRuntime() < startTime + 0.5) {}
        robot.arm.extend(0);
        //take video of sampling field, find most commonly detected gold position
        robot.drivetrain.turn(50);
        double heading = robot.drivetrain.heading();
        contourVision.enable();
        int goldCount = 0;
        while(goldCount < 5 && Math.abs(robot.drivetrain.heading() - heading) < 100) {
            if(contourVision.getGoldPos()) {
                robot.drivetrain.slow(0.4);
                goldCount++;
            } else {
                robot.drivetrain.slow(1.0);
            }
        }
        contourVision.disable();

        //fold slides
        robot.arm.swing(false);
        //drive through gold position
        robot.drivetrain.driveEnc(6);
        robot.drivetrain.whileBusy();
        robot.drivetrain.driveEnc(-6);
        robot.drivetrain.whileBusy();
        //drive to depot
        robot.drivetrain.turn((int)(heading - robot.drivetrain.heading()));
        robot.drivetrain.whileBusy();
        robot.drivetrain.turn(90);
        robot.drivetrain.whileBusy();
        robot.drivetrain.driveEnc(24 * Math.sqrt(2));
        robot.drivetrain.whileBusy();
        robot.drivetrain.turn(-45);
        robot.drivetrain.whileBusy();
        robot.drivetrain.driveEnc(60);
        robot.drivetrain.whileBusy();
        //drop team marker
        robot.intake.intake(-1);
        sleep(500);
        robot.intake.stop();
        //park
        robot.drivetrain.turn(180);
        robot.drivetrain.whileBusy();
        robot.drivetrain.driveEnc(72);
        robot.drivetrain.whileBusy();

        robot.drivetrain.stop();
    }
}