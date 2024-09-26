package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.RobotConstantsIntoTheDeep;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.auto.FTCAuto;
import org.firstinspires.ftc.teamcode.common.FTCErrorHandling;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;

// Use this dispatcher class to place the launching of FTCAuto and all of the error
// handling in one place.
public class FTCAutoDispatch {

    public static void runAuto(RobotConstants.RunType pRunType,
                               RobotConstantsIntoTheDeep.OpMode pOpMode,
                               RobotConstants.Alliance pAlliance,
                               LinearOpMode pLinear) throws InterruptedException {

        final String TAG = FTCAutoDispatch.class.getSimpleName();

        pLinear.telemetry.setAutoClear(false); // keep our messages on the driver station

        // LCHSAuto, the common class for all autonomous opmodes, needs
        // access to the public data fields and methods in LinearOpMode.
        try {
            RobotLogCommon.initialize(RobotLogCommon.LogIdentifier.AUTO_LOG, WorkingDirectory.getWorkingDirectory() + RobotConstants.LOG_DIR);

            RobotLogCommon.c(TAG, "Constructing FTCRobot with run type " + pRunType);
            FTCRobot robot = new FTCRobot(pLinear, pRunType);
            FTCAuto ftcAuto = new FTCAuto(pAlliance, pLinear, robot, pOpMode);

            pLinear.telemetry.addData(TAG, "Waiting for start ...");
            pLinear.telemetry.update();

            //?? Where does control go if there's a crash in init?
            // The exception message appears on the Driver Station,
            // it may or may not be written to the FTCAutoLog (not
            // (not time?) but it is always written to the match log,
            // e.g.
            // 12-27 16:34:06.616  1673  2774 E FTCAutoDispatch:  ** FATAL Java Exception ** expected: /BORWARD read: FORWARD (position:END_TAG </FORWARD>@16:23 in java.io.InputStreamReader@d2b7e51)
            pLinear.waitForStart();

            //## 12/28/2022 Note: if initialization is complete and the play
            // button is showing we're in waitForStart(); if the driver hits
            // the small stop button, isOpModeActive() returns false but
            // runRobot() will be called. See further notes at that location.
            RobotLogCommon.i(TAG, "After waitForStart()");
            pLinear.telemetry.addData(TAG, "Running ...");
            pLinear.telemetry.update();

            ftcAuto.runRobot();
        } catch (Exception ex) {
            FTCErrorHandling.handleFtcErrors(ex, TAG, pLinear);
        } finally {
            RobotLogCommon.closeLog();

            // New in FTC SDK 7.2.
            pLinear.terminateOpModeNow();
        }
    }
}
