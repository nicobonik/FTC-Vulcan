package org.firstinspires.ftc.teamcode.Subsystems;

import java.util.LinkedHashMap;

public class NewArm extends Subsystem {
    private final double linkageLength = 6;
    private final double ticksPerRotatorInch = 103.6 * 0.125 * 25.4;
    private final double ticksPerExtenderInch = 537.6;
    public LinkedHashMap<String, String> updateSubsystem() {

        return telemetryPackets;
    }
}
