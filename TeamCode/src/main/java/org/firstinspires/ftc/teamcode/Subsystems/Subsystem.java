package org.firstinspires.ftc.teamcode.Subsystems;

import java.util.LinkedHashMap;
import java.util.Map;

public abstract class Subsystem {
    LinkedHashMap<String, String> telemetryPackets;
    public abstract LinkedHashMap<String, String> updateSubsystem();
}
