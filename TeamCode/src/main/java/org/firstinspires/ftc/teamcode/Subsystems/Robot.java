package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.LinkedHashMap;

import static org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity.TAG;

public class Robot {
    public Drivetrain drivetrain;
    public Intake intake;
    public Arm arm;
    private Subsystem[] subsystems;
    private Runnable subsystemUpdater, telemetryUpdater;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Thread subsystemUpdateThread, telemetryUpdateThread;
    private LinkedHashMap<String, String> telemetryPackets;

    public Robot(HardwareMap hwMap, Telemetry telem) {
        telemetry = telem;
        telemetryPackets = new LinkedHashMap<>();
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
        subsystemUpdater = new Runnable() {
            public void run() {
                try {
                    Thread.sleep(100);
                    while (!Thread.currentThread().isInterrupted()) {
                        for (Subsystem subsystem : subsystems) {
                            if (subsystem != null) {
                                telemetryPackets.putAll(subsystem.updateSubsystem());
                            }
                        }
                    }
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    stop();
                }
            }
        };
        /*telemetryUpdater = new Runnable() {
            public void run() {
                try {
                    Thread.sleep(100);
                    while (!Thread.currentThread().isInterrupted()) {
                        for (String key : telemetryPackets.keySet()) {
                            telemetry.addData(key, telemetryPackets.get(key));
                        }
                    }
                } catch (InterruptedException e) {
                    telemetry.addData("interrupted", true);
                    telemetry.update();
                    Thread.currentThread().interrupt();
                    stop();
                }
            }
        };*/
    }

    public void init() {
        subsystemUpdateThread = new Thread(subsystemUpdater);
        subsystemUpdateThread.start();
        //telemetryUpdateThread = new Thread(telemetryUpdater);
        //telemetryUpdateThread.start();
    }

    public void stop() {
        subsystemUpdateThread.interrupt();
        //telemetryUpdateThread.interrupt();
        drivetrain.stop();
        arm.stop();
        intake.stop();
    }
}
