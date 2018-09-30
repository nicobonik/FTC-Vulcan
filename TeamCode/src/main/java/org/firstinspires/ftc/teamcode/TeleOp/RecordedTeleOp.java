package org.firstinspires.ftc.teamcode.TeleOp;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.FileOutputStream;

@TeleOp(name="RecordDrive", group="Test")
public class RecordedTeleOp extends Tele {
    private FileOutputStream outputStream;
    public void init() {
        super.init();
        try {
            outputStream = hardwareMap.appContext.openFileOutput("recordedTeleop", Context.MODE_PRIVATE);
        } catch(Exception e) {
            e.printStackTrace();
            requestOpModeStop();
        }
    }

    public void loop() {
        super.loop();
        StringBuilder state = new StringBuilder();
        state.append(drivetrain.speeds());
        for(boolean key : keys) {
            state.append(key ? 1 : 0);
        }
        state.append(getRuntime() + "\n");
        try {
            outputStream.write(state.toString().getBytes());
        } catch(Exception e) {
            e.printStackTrace();
            requestOpModeStop();
        }
    }
    public void stop() {
        super.stop();
        try {
            outputStream.close();
        } catch(Exception e) {
            e.printStackTrace();
            requestOpModeStop();
        }
    }
}
