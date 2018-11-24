package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity.TAG;

public class Robot {
    public Drivetrain drivetrain;
    public Intake intake;
    public Arm arm;
    public Subsystem[] subsystems;
    public Runnable subsystemUpdater;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    private Thread subsystemUpdateThread;

    public Robot(HardwareMap hwMap) {
        hardwareMap = hwMap;
        drivetrain = new Drivetrain(
                hardwareMap.dcMotor.get("front_left"),
                hardwareMap.dcMotor.get("front_right"),
                hardwareMap.dcMotor.get("back_left"),
                hardwareMap.dcMotor.get("back_right"),
                hardwareMap.get(BNO055IMU.class, "imu")
        );
        arm = new Arm(
                new DcMotor[] {hardwareMap.dcMotor.get("arm_left"), hardwareMap.dcMotor.get("arm_right")},
                hardwareMap.dcMotor.get("extender")//,
                //hardwareMap.analogInput.get("potent")
        );
        intake = new Intake(
                hardwareMap.crservo.get("intake"),
                hardwareMap.servo.get("door")
        );
        subsystems = new Subsystem[] {drivetrain, intake, arm};

        /*subsystemUpdater = new Runnable() {
            public void run() {
                try {
                    while (!Thread.currentThread().isInterrupted()) {
                        //TelemetryPacket packet;
                        for (Subsystem subsystem : subsystems) {
                            if (subsystem != null) {
                                subsystem.updateSubsystem();
                            }
                            //packet = subsystem.updateSubsystem();
                        /*for (Listener listener : listeners) {
                            listener.onPostUpdate();
                        }*/
                        /*while (telemetryPacketQueue.remainingCapacity() == 0) {

                        }
                        telemetryPacketQueue.add(packet);
                        }
                        Thread.sleep(1);
                    }
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        };*/
    }

    public Robot(HardwareMap hwMap, Telemetry telem) {
        telemetry = telem;
        hardwareMap = hwMap;
        //telemetry.addData("drivetrain", "initializing");
        //telemetry.update();
        drivetrain = new Drivetrain(
                hardwareMap.dcMotor.get("front_left"),
                hardwareMap.dcMotor.get("front_right"),
                hardwareMap.dcMotor.get("back_left"),
                hardwareMap.dcMotor.get("back_right"),
                hardwareMap.get(BNO055IMU.class, "imu")
        );
        //telemetry.addData("drivetrain", "initialized");
        //telemetry.addData("arm", "initializing");
        //telemetry.update();
        arm = new Arm(
            new DcMotor[] {hardwareMap.dcMotor.get("arm_left"), hardwareMap.dcMotor.get("arm_right")},
            hardwareMap.dcMotor.get("extender")//,
            //hardwareMap.analogInput.get("potent")
        );
        //telemetry.addData("arm", "initialized");
        //telemetry.addData("intake", "initializing");
        //telemetry.update();
        intake = new Intake(
            hardwareMap.crservo.get("intake"),
            hardwareMap.servo.get("door")
        );
        //telemetry.addData("intake", "initialized");
        //telemetry.addData("runnable", "initializing");
        //telemetry.update();
        subsystems = new Subsystem[] {drivetrain, intake, arm};
        subsystemUpdater = new Runnable() {
            public void run() {
                try {
                    while (!Thread.currentThread().isInterrupted()) {
                        for (Subsystem subsystem : subsystems) {
                            if (subsystem != null) {
                                subsystem.updateSubsystem();
                            }
                        }
                        Thread.sleep(1);
                    }
                } catch (InterruptedException e) {
                    telemetry.addData("interrupted", true);
                    telemetry.update();
                    Thread.currentThread().interrupt();
                    stop();
                }
            }
        };
        //telemetry.addData("runnable", "initialized");
        //telemetry.update();
    }

    public void init() {
        subsystemUpdateThread = new Thread(subsystemUpdater);
        subsystemUpdateThread.start();
    }

    public void stop() {
        subsystemUpdateThread.interrupt();
        drivetrain.stop();
        arm.stop();
        intake.stop();
    }
}
