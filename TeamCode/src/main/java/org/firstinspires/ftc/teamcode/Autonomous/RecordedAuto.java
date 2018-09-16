package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.TeleOp.Tele;

import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.InputStreamReader;

@Autonomous(name = "RecordedAuto", group = "Drive")
public class RecordedAuto extends Tele {
    private FileInputStream inStream;
    private String[] split;
    private BufferedReader buf;
    public void init() {
        super.init();
        try {
            inStream = hardwareMap.appContext.openFileInput("recordedTeleop");
            buf = new BufferedReader(new InputStreamReader(inStream));
        } catch (Exception e) {
            e.printStackTrace();
            requestOpModeStop();
        }
    }

    public void loop() {
        try {
            double time = 0;
            String line = buf.readLine();
            String[] last = line.split(" ");
            while(time < getRuntime()) {
                line = buf.readLine();
                if (line != null) {
                    split = line.split(" ");
                    time = Double.parseDouble(split[5]);
                } else {
                    stop();
                }
            }
            double[] motorPowers = new double[4];
            for(int i = 0; i < 4; i++) {
                motorPowers[i] = Double.parseDouble(last[i]);
            }
            drivetrain.speeds(motorPowers);
            super.loop();
        } catch(Exception e) {
            e.printStackTrace();
            requestOpModeStop();
        }
    }

    public void stop() {
        super.stop();
        try {
            inStream.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void getKeys() {
        char[] keyCodes = split[4].toCharArray();
        for (int code = 0; code < keyCodes.length; code++) {
            keys[code] = keyCodes[code] == '1';
        }
    }

    protected void arcadeDrive(double forward, double turn) {}
}
