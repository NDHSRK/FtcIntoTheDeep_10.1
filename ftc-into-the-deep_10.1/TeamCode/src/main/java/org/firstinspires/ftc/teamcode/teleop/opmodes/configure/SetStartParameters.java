package org.firstinspires.ftc.teamcode.teleop.opmodes.configure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsIntoTheDeep;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.firstinspires.ftc.teamcode.teleop.common.FTCButton;
import org.firstinspires.ftc.teamcode.teleop.common.FTCToggleButtonNWay;
import org.firstinspires.ftc.teamcode.xml.StartParameters;
import org.firstinspires.ftc.teamcode.xml.StartParametersXML;
import org.xml.sax.SAXException;

import java.io.IOException;
import java.util.EnumSet;

import javax.xml.parsers.ParserConfigurationException;

// A class that supports two sets of start parameters, a STANDARD
// set with its particular button layout and an optional set with
// a different button layout. A toggle button switches back and
// forth between the two sets.

//## The CenterStageCore project does not contain any optional parameters
// because they are game-specific. But skeleton support for optional
// remains as a placeholder.

@TeleOp(name = "SetStartParameters", group = "Configure")
//@Disabled
public class SetStartParameters extends LinearOpMode {

    private static final String TAG = SetStartParameters.class.getSimpleName();

    private StartParametersXML startParametersXML;
    private StartParameters startParameters;
    private boolean startParametersXMLChanged = false;

    private static final int MAX_DELAY = 10; // applies to all delays

    private enum Mode {STANDARD, OPTIONAL}

    // Toggle button for switching between the STANDARD button layout
    // and the OPTIONAL button layout.
    private FTCToggleButtonNWay<Mode> toggleMode;

    // This section applies to the STANDARD mode only
    private int currentStartDelay;
    RobotConstantsIntoTheDeep.OpMode currentOpMode = RobotConstantsIntoTheDeep.OpMode.OPMODE_NPOS;
    private FTCButton factoryReset;
    private boolean factoryResetRequested = false;
    private boolean factoryResetExecuted = false;
    // End STANDARD mode section

    // This section applies to the modal use of the same buttons
    private FTCButton modalIncreaseDelay; // STANDARD increase start delay, OPTIONAL increase mid-point delay
    private FTCButton modalDecreaseDelay; // STANDARD decrease start delay, OPTIONAL decrease mid-point delay
    private FTCButton modalButton1X; // factory reset cancel
    private FTCButton modalButton1Y; // factory reset ok
    // End modal section

    @Override
    public void runOpMode() {
        RobotLogCommon.c(TAG, "Initializing SetStartParameters");
        // Toggle button for switching between STANDARD mode and OPTIONAL mode.
        // The STANDARD button layout is the starting toggle position. Use
        // GAMEPAD_1_LEFT_BUMPER to switch to the OPTIONAL layout. Continue
        // using GAMEPAD_1_LEFT_BUMPER to alternate.
        toggleMode = new FTCToggleButtonNWay<>(this, FTCButton.ButtonValue.GAMEPAD_1_LEFT_BUMPER,
               EnumSet.allOf(Mode.class));

        // Button assignments for STANDARD mode.
        factoryReset = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_START);

        // Button assignments for OPTIONAL mode.
        //## Optional button assignments go here.

        // Modal button assignments.
        modalIncreaseDelay = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_DPAD_UP);
        modalDecreaseDelay = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_DPAD_DOWN);
        modalButton1X = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_X);
        modalButton1Y = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_Y);

        initializeStartParameters();

        while (!isStarted() && !isStopRequested()) {
            updateButtons();
            updatePlayer1();
            updateTelemetry();
            sleep(50); // Don't burn CPU cycles busy-looping
        } // while

        if (opModeIsActive()) {
            if (startParametersXMLChanged) {
                startParametersXML.writeStartParametersFile();
                RobotLogCommon.i(TAG, "Writing StartParameters.xml");
                telemetry.addLine("Writing StartParameters.xml");
            } else
                // Do not output if factory reset has been executed.
                if (!factoryResetExecuted)
                    telemetry.addLine("No changes to StartParameters.xml");

            telemetry.update();
            sleep(1500);
        }
    }

    private void updateButtons() {
        // If a factory reset has been requested only update the
        // buttons for confirm and cancel.
        if (factoryResetRequested) {
            modalButton1Y.update();
            modalButton1X.update();
            return;
        }

        toggleMode.update();
        modalIncreaseDelay.update();
        modalDecreaseDelay.update();
        factoryReset.update();
    }

    private void updatePlayer1() {
        // If a factory reset has been requested only test for
        // confirm or cancel.
        if (factoryResetRequested) {
            if (!updateFactoryResetConfirm())
                updateFactoryResetCancel();
            return;
        }

        updateMode();

        if (toggleMode.getToggleState() == Mode.STANDARD) {
            updateIncreaseStartDelay();
            updateDecreaseStartDelay();
            updateFactoryReset();
        } else { // Mode.OPTIONAL
            //## Updates for optional parameters go here.
        }
    }

    private void updateMode() {
        if (toggleMode.is(FTCButton.State.TAP)) {
            toggleMode.toggle();
            if (toggleMode.getToggleState() == Mode.OPTIONAL) {
                //## If you want to use optional parameters, they must be present in the XML file.
                //if (startParameters.optionalStartParameters == null) {
                // No good, toggle back to STANDARD.
                //  toggleMode.toggle();
                //  telemetry.addLine("Toggle disallowed: optional elements are not present in StartParameters.xml");
                //  telemetry.update();
                // return;
                // }
            }
        }
    }

    // STANDARD mode methods.
    private void updateIncreaseStartDelay() {
        if (modalIncreaseDelay.is(FTCButton.State.TAP)) {
            if (currentStartDelay < MAX_DELAY) {
                ++currentStartDelay;
                startParametersXML.setAutoStartDelay(currentStartDelay);
                startParametersXMLChanged = true;
            }
        }
    }

    private void updateDecreaseStartDelay() {
        if (modalDecreaseDelay.is(FTCButton.State.TAP)) {
            if (currentStartDelay > 0) {
                --currentStartDelay;
                startParametersXML.setAutoStartDelay(currentStartDelay);
                startParametersXMLChanged = true;
            }
        }
    }

    private void updateFactoryReset() {
        if (factoryReset.is(FTCButton.State.TAP)) {
            factoryResetRequested = true;
            factoryResetExecuted = false;
        }
    }

    private boolean updateFactoryResetConfirm() {
        if (!modalButton1Y.is(FTCButton.State.TAP))
            return false;

        factoryResetRequested = false;
        factoryResetExecuted = true;

        // Call all XML set... methods and reset the values to their defaults.
        startParametersXML.setAutoStartDelay(0);

        // Write out the XML file.
        startParametersXML.writeStartParametersFile();
        RobotLogCommon.i(TAG, "Writing StartParameters.xml");
        telemetry.addLine("Writing StartParameters.xml");
        telemetry.update();
        sleep(1500);

        // Read the XML file back in.
        initializeStartParameters();
        return true;
    }

    private void updateFactoryResetCancel() {
        if (modalButton1X.is(FTCButton.State.TAP))
            factoryResetRequested = false;
    }

    private void initializeStartParameters() {
        startParametersXMLChanged = false;

        // Reinitialize toggle button to its starting STANDARD position.
        toggleMode.setToggleState(Mode.STANDARD);
        currentOpMode = RobotConstantsIntoTheDeep.OpMode.OPMODE_NPOS;

        String fullXMLDir = WorkingDirectory.getWorkingDirectory() + RobotConstants.XML_DIR;
        try {
            startParametersXML = new StartParametersXML(fullXMLDir);
        } catch (ParserConfigurationException | SAXException | IOException e) {
            throw new AutonomousRobotException(TAG, e.getMessage());
        }

        startParameters = startParametersXML.getStartParameters();
        currentStartDelay = startParameters.autoStartDelay;
    }

    private void updateTelemetry() {
        if (toggleMode.getToggleState() == Mode.STANDARD) {
            telemetry.addLine("The current mode is STANDARD");

            // If a factory reset has been requested only show the confirm/cancel options.
            if (factoryResetRequested) {
                telemetry.addLine("Factory reset requested");
                telemetry.addLine("  Press Y to confirm, X to cancel");
                telemetry.update();
                return;
            }

            telemetry.addLine("The current start delay is " + currentStartDelay);
            telemetry.addLine("Press DPAD_UP to increase delay; DPAD_DOWN to decrease");
            telemetry.addLine("The current OpMode is " + currentOpMode);
            telemetry.addLine("Press START for factory reset");
        } else { // Mode.OPTIONAL
            //## Telemetry for the optional parameters goes here.
        }

        telemetry.addLine("Touch play to SAVE changes and END the OpMode");
        telemetry.update();
    }

}