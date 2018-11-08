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

@Autonomous(name = "TestAutonomous", group="Auto")
public class Auto extends LinearOpMode{
    private Drivetrain drivetrain;
    private Arm arm;
    private Intake intake;
    private ElapsedTime Runtime = new ElapsedTime();
    private MineralVisionHough houghVision;
    private int[] goldPos = new int[3];
    public void runOpMode() {
        drivetrain = new Drivetrain(
                hardwareMap.dcMotor.get("front_left"),
                hardwareMap.dcMotor.get("front_right"),
                hardwareMap.dcMotor.get("back_left"),
                hardwareMap.dcMotor.get("back_right"),
                hardwareMap.get(BNO055IMU.class, "imu")
        );
        arm = new Arm(new DcMotor[] {hardwareMap.dcMotor.get("arm_left"), hardwareMap.dcMotor.get("arm_right")}, hardwareMap.dcMotor.get("extender"));
        intake = new Intake(hardwareMap.dcMotor.get("intake"), hardwareMap.servo.get("door"));
        houghVision = new MineralVisionHough();
        houghVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        houghVision.setShowCountours(false);
        houghVision.setTelem(telemetry);
        waitForStart();
        Runtime.reset();
        //unfold bot
        arm.swing(true);
        arm.whileBusy();
        //extend slides
        arm.extendDist(10);
        arm.whileBusy();
        //drive forward
        drivetrain.resetOrientation(); //reset IMU because robot has landed
        drivetrain.driveEnc(4);
        drivetrain.whileBusy();
        //retract slides
        arm.extend(false);
        //fold slides
        arm.swing(false);
        //take video of sampling field, find most commonly detected gold position
        houghVision.enable();
        for(int i = 0; i < 10; i++) {
            goldPos[houghVision.getGoldPos()]++;
        }
        int max = 0;
        int argmax = 3;
        for(int i = 0; i < 2; i++) {
            if(goldPos[i] > max) {
                max = goldPos[i];
                argmax = i;
            }
        }
        //drive through gold position
        /*drivetrain.turn(45 * (argmax - 1));
        drivetrain.whileBusy();
        drivetrain.driveEnc(6);
        drivetrain.whileBusy();
        //drive to depot
        drivetrain.turn(-90 * (argmax - 1));
        drivetrain.whileBusy();
        drivetrain.driveEnc(6);
        drivetrain.whileBusy();
        drivetrain.turn(45 * (argmax - 1));
        drivetrain.whileBusy();
        drivetrain.driveEnc(24);
        drivetrain.whileBusy();
        //drop team marker
        intake.intake(-1);
        sleep(500);
        intake.stop();
        //park
        drivetrain.turn(135);
        drivetrain.whileBusy();
        drivetrain.driveEnc(48);
        drivetrain.whileBusy();*/
        drivetrain.stop();
    }
}