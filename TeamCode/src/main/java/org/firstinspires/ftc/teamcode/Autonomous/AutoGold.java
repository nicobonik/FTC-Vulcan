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
        super.runOpMode();
        robot.drivetrain.turnEnc(50);
        while(robot.drivetrain.isBusy() && opModeIsActive()) {}
        runUntilGold();
        robot.arm.swingAngle(0);
        while(robot.arm.isBusy() && opModeIsActive()) {}
        robot.drivetrain.driveEnc(-6);
        while(robot.drivetrain.isBusy() && opModeIsActive()) {}
        robot.drivetrain.turn(-robot.drivetrain.heading() + 90);
        while(robot.drivetrain.isBusy() && opModeIsActive()) {}
        robot.drivetrain.driveEnc(48 * Math.sqrt(2));
        while(robot.drivetrain.isBusy() && opModeIsActive()) {}
        robot.drivetrain.turn(45);
        while(robot.drivetrain.isBusy() && opModeIsActive()) {}
        robot.drivetrain.driveEnc(24);
        while(robot.drivetrain.isBusy() && opModeIsActive()) {}
        //placeholder for drop marker
        robot.drivetrain.turn(180);
        while(robot.drivetrain.isBusy() && opModeIsActive()) {}
        robot.drivetrain.driveEnc(72);
        while(robot.drivetrain.isBusy() && opModeIsActive()) {}
        robot.arm.extendDist(4);
        robot.arm.swingAngle(15);
        while(robot.arm.isBusy() && opModeIsActive()) {}
        robot.arm.swingAngle(0);
        vis.disable();
        robot.stop();
    }

    private void runUntilGold() {
        while (!vis.getGoldPos()) {
            robot.drivetrain.turnEnc(reSweep ? 5 : -5);
            sweepProgress++;
            if(sweepProgress > 20) {
                reSweep = !reSweep;
                sweepProgress = 0;
            }
            while (robot.drivetrain.isBusy() && opModeIsActive()) {}
        }
        int counter = 0;
        for(int i = 0; i < 10; i++) {
            if(vis.getGoldPos()) {
                counter++;
            }
        }
        if(counter < 7) {
            runUntilGold();
        }
    }
}