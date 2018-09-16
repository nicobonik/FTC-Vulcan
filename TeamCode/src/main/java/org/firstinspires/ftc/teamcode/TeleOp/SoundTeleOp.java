package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.SoundPlayer;

@TeleOp(name="TestSound", group="Test")
public class SoundTeleOp extends OpMode {
    boolean last = false;
    public void init() {
    }

    public void loop() {
        if(gamepad1.a && !last) {
            SoundPlayer.start(hardwareMap.appContext);
            last = true;
        } else {
            SoundPlayer.stop();
        }
    }

    public void stop() {
    }
}
