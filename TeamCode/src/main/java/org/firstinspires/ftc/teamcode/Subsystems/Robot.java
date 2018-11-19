package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity.TAG;

public class Robot {
    public Drivetrain drivetrain;
    public Intake intake;
    public Arm arm;
    public Subsystem[] subsystems = {drivetrain, intake, arm};
    public Runnable subsystemUpdater;

    public Robot(HardwareMap hardwareMap) {
        drivetrain = new Drivetrain(
                hardwareMap.dcMotor.get("front_left"),
                hardwareMap.dcMotor.get("front_right"),
                hardwareMap.dcMotor.get("back_left"),
                hardwareMap.dcMotor.get("back_right"),
                hardwareMap.get(BNO055IMU.class, "imu")
        );
        arm = new Arm(
                new DcMotor[] {hardwareMap.dcMotor.get("arm_left"), hardwareMap.dcMotor.get("arm_right")},
                hardwareMap.dcMotor.get("extender"),
                hardwareMap.analogInput.get("potent")
        );
        intake = new Intake(
                hardwareMap.dcMotor.get("intake"),
                hardwareMap.servo.get("door")
        );

        subsystemUpdater = new Runnable() {
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
                        telemetryPacketQueue.add(packet);*/
                        }
                        Thread.sleep(1);
                    }
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();

                }
            }
        };
    }

    public void init() {
        new Thread(subsystemUpdater).start();
    }

    public void stop() {
        drivetrain.stop();
        arm.stop();
        intake.stop();
    }
}
