package org.firstinspires.ftc.teamcode.Subsystems;
import android.content.Context;
import android.media.MediaPlayer;

import org.firstinspires.ftc.teamcode.R;

import java.io.IOException;

public class SoundPlayer {
    private static MediaPlayer player;
    public static void start(Context context) {
        if(player == null) {
            player = MediaPlayer.create(context, R.raw.anthem);
            player.seekTo(0);
            player.start();
        }
    }

    public static void stop() {
        if (player != null) {
            player.stop();
            try {
                player.prepare();
            }
            catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
}
