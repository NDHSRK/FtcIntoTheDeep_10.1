package org.firstinspires.ftc.teamcode.xml;

// Starting parameters that can be set via a dedicated TeleOp
// OpMode and read by either an Autonomous or TeleOp competition
// OpMode.
public class StartParameters {

    public final String robotConfigFilename;
    public final String robotActionFilename;
    public final int autoStartDelay;

    public StartParameters(String pRobotConfigFilename, String pRobotActionFilename, int pAutoStartDelay) {
        robotConfigFilename = pRobotConfigFilename;
        robotActionFilename = pRobotActionFilename;
        autoStartDelay = pAutoStartDelay;
    }

}