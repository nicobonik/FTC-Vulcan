package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.SoundPlayer;

import java.util.Random;

@TeleOp(name="TestDrive", group="Drive")
public class Tele extends OpMode {
    protected Drivetrain drivetrain;
    private boolean unlocked = false;
    private Random r = new Random();
    private int[][] sequences = {
            {1, 5, 5, 7, 7, 6, 4, 6, 4, 1, 0, 12}
    };
    protected boolean[] keys = new boolean[16];
    private boolean[] keyPressed = new boolean[15];

    public void init() {
        drivetrain = new Drivetrain(
                hardwareMap.dcMotor.get("front_left"),
                hardwareMap.dcMotor.get("front_right"),
                hardwareMap.dcMotor.get("back_left"),
                hardwareMap.dcMotor.get("back_right"),
                hardwareMap.get(BNO055IMU.class, "imu")
        );
        gamepad1.setJoystickDeadzone(0.05f);
    }

    public void loop() {
        getKeys();
        if(keyPressed[0]) {
            SoundPlayer.start(hardwareMap.appContext);
        }
        if(keys[9]) {
            drivetrain.tempPower = Drivetrain.BASE_POWER/2;
        } else {
            drivetrain.tempPower = Drivetrain.BASE_POWER;
        }
        if(keys[11]) {
            drivetrain.turn(90);
        } else if(keys[10]) {
            drivetrain.turn(-90);
        } else {
            arcadeDrive(gamepad1.left_stick_y, gamepad1.right_stick_x);
        }
        if(keys[15]) {
            checkSequences();
        }
        if(!unlocked) {
            telemetry.addData("locked", true);
            telemetry.update();
        }
        /*if(!unlocked && r.nextDouble() > 0.99) {
            drivetrain.turnEnc((int) (r.nextDouble() * 90));
        }*/
    }

    public void stop() {
        drivetrain.stop();
    }

    protected void getKeys() {
        keyPressed = new boolean[15];
        if(gamepad1.a) {
            if(!keys[0]) {
                keyPressed[0] = true;
            }
            keys[0] = true;
        } else {
            keys[0] = false;
        }
        if(gamepad1.b) {
            if(!keys[1]) {
                keyPressed[1] = true;
            }
            keys[1] = true;
        } else {
            keys[1] = false;
        }
        if(gamepad1.x) {
            if(!keys[2]) {
                keyPressed[2] = true;
            }
            keys[2] = true;
        } else {
            keys[2] = false;
        }
        if(gamepad1.y) {
            if(!keys[3]) {
                keyPressed[3] = true;
            }
            keys[3] = true;
        } else {
            keys[3] = false;
        }
        if(gamepad1.dpad_right) {
            if(!keys[4]) {
                keyPressed[4] = true;
            }
            keys[4] = true;
        } else {
            keys[4] = false;
        }
        if(gamepad1.dpad_up) {
            if(!keys[5]) {
                keyPressed[5] = true;
            }
            keys[5] = true;
        } else {
            keys[5] = false;
        }
        if(gamepad1.dpad_left) {
            if(!keys[6]) {
                keyPressed[6] = true;
            }
            keys[6] = true;
        } else {
            keys[6] = false;
        }
        if(gamepad1.dpad_down) {
            if(!keys[7]) {
                keyPressed[7] = true;
            }
            keys[7] = true;
        } else {
            keys[7] = false;
        }
        if(gamepad1.left_bumper) {
            if(!keys[8]) {
                keyPressed[8] = true;
            }
            keys[8] = true;
        } else {
            keys[8] = false;
        }
        if(gamepad1.right_bumper) {
            if(!keys[9]) {
                keyPressed[9] = true;
            }
            keys[9] = true;
        } else {
            keys[9] = false;
        }
        if(gamepad1.left_trigger > 0.2) {
            if(!keys[10]) {
                keyPressed[10] = true;
            }
            keys[10] = true;
        } else {
            keys[10] = false;
        }
        if(gamepad1.right_trigger > 0.2) {
            if(!keys[11]) {
                keyPressed[11] = true;
            }
            keys[11] = true;
        } else {
            keys[11] = false;
        }
        if(gamepad1.start) {
            if(!keys[12]) {
                keyPressed[12] = true;
            }
            keys[12] = true;
        } else {
            keys[12] = false;
        }
        if(gamepad1.back) {
            if(!keys[13]) {
                keyPressed[13] = true;
            }
            keys[13] = true;
        } else {
            keys[13] = false;
        }
        if(gamepad1.left_stick_button) {
            if(!keys[14]) {
                keyPressed[14] = true;
            }
            keys[14] = true;
        } else {
            keys[14] = false;
        }
        if(gamepad1.right_stick_button) {
            if(!keys[15]) {
                keyPressed[15] = true;
            }
            keys[15] = true;
        } else {
            keys[15] = false;
        }
        keys[16] = gamepad1.guide;
    }

    private void checkSequences() {
        boolean anyKey = false;
        for(boolean key : keyPressed) {
            if(key) {
                anyKey = true;
            }
        }
        for(int sequence = 0; sequence < sequences.length; sequence++) {
            if(keyPressed[sequences[sequence][sequences[sequence][0]]]) {
                sequences[sequence][0]++;
                if(sequences[sequence][0] == sequences[sequence].length) {
                    routines[sequence].execute();
                }
            } else if(!keys[sequences[sequence][sequences[sequence][0]]] && anyKey) {
                sequences[sequence][0] = 1;
            }
        }
    }

    protected void arcadeDrive(double forward, double turn) {
        drivetrain.arcadeDrive(forward, turn);
    }

    private interface routine {
        void execute();
    }
    private routine[] routines = new routine[] {
        new routine() {
            public void execute() {
                unlocked = true;
            }
        }
    };
}