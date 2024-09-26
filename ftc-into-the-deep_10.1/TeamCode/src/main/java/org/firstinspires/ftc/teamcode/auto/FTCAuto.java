package org.firstinspires.ftc.teamcode.auto;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.auto.vision.AprilTagUtils.AprilTagId.getEnumValue;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.android.TimeStamp;
import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.ftcdevcommon.xml.RobotXMLElement;
import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;
import org.firstinspires.ftc.teamcode.auto.vision.AprilTagUtils;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsIntoTheDeep;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.robot.device.camera.AprilTagAccess;
import org.firstinspires.ftc.teamcode.robot.device.camera.RawFrameProcessor;
import org.firstinspires.ftc.teamcode.robot.device.camera.VisionPortalWebcam;
import org.firstinspires.ftc.teamcode.robot.device.camera.VisionPortalWebcamConfiguration;
import org.firstinspires.ftc.teamcode.robot.device.motor.ElevatorMotion;
import org.firstinspires.ftc.teamcode.robot.device.motor.SingleMotorMotion;
import org.firstinspires.ftc.teamcode.xml.RobotActionXML;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.xml.sax.SAXException;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Date;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.List;
import java.util.Objects;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPathException;
import javax.xml.xpath.XPathExpressionException;

public class FTCAuto {

    private static final String TAG = FTCAuto.class.getSimpleName();

    private final RobotConstants.Alliance alliance;
    private final LinearOpMode linearOpMode;
    private final FTCRobot robot;
    private final String workingDirectory;
    private final int autoStartDelay;
    private final RobotActionXML.RobotActionData actionData;
    private final MecanumDrive roadrunnerDrive;

    private enum ActionStatus {
        NO_ACTIONS_TAKEN, OPMODE_INACTIVE, OPMODE_TIMEOUT,
        ACTION_STOP, ACTION_STOP_ALL, ACTION_COMPLETE, ALL_ACTIONS_COMPLETE
    }

    private final EnumSet<RobotConstantsIntoTheDeep.InternalWebcamId> openWebcams = EnumSet.noneOf(RobotConstantsIntoTheDeep.InternalWebcamId.class);

    public static AutonomousTimer autonomousTimer; // grant visibility to all of Autonomous

    // Main class for the autonomous run.
    public FTCAuto(RobotConstants.Alliance pAlliance, LinearOpMode pLinearOpMode, FTCRobot pRobot,
                   RobotConstantsIntoTheDeep.OpMode pOpMode)
            throws ParserConfigurationException, SAXException, XPathException, IOException {

        RobotLogCommon.c(TAG, "Constructing FTCAuto with alliance " + pAlliance + " running OpMode " + pOpMode);

        alliance = pAlliance;
        linearOpMode = pLinearOpMode; // FTC context
        robot = pRobot; // robot hardware

        // Get the directory for the various configuration files.
        workingDirectory = WorkingDirectory.getWorkingDirectory();
        String xmlDirectory = workingDirectory + RobotConstants.XML_DIR;

        // Get the configurable delay at Autonomous startup.
        autoStartDelay = robot.startParameters.autoStartDelay;

        // Read the RobotAction XXX.xml file for all OpModes.
        RobotLogCommon.c(TAG, "Getting the Autonomous choreographies from " + robot.startParameters.robotActionFilename);
        RobotActionXML actionXML = new RobotActionXML(robot.startParameters.robotActionFilename);

        // Extract data from the parsed XML file for the selected OpMode only.
        actionData = actionXML.getOpModeData(pOpMode.toString());

        // Initialize the hardware and classes that control motion.
        // Do not initialize if the components have been configured out.

        RobotLogCommon.CommonLogLevel logLevel = actionData.logLevel;
        RobotLogCommon.setMostDetailedLogLevel(logLevel);
        RobotLogCommon.c(TAG, "Most detailed log level " + RobotLogCommon.getMostDetailedLogLevel());

        // If a drive train is part of the current configuration, initialize Roadrunner.
        if (robot.roadrunnerDriveTrainInConfiguration) {
            if (actionData.startingPositionData == null)
                throw new AutonomousRobotException(TAG, "Missing Roadrunner initial pose");

            roadrunnerDrive = new MecanumDrive(pLinearOpMode.hardwareMap, new Pose2d(actionData.startingPositionData.x,
                    actionData.startingPositionData.y, Math.toRadians(actionData.startingPositionData.angle)));
        } else
            roadrunnerDrive = null;

        // Start the front webcam with the raw webcam frame processor.
        // We can start a camera by using the <START_CAMERA> action in RobotAction.xml
        // but since it's typical that the first task in Autonomous requires image
        // recognition, we save time by starting the front webcam here with two
        // processors: one for raw frames (enabled) and one for AprilTags (disabled)
        // OR a single processor for raw frames (enabled). The only time this camera
        // might not be in the configuration is during testing of the drive train
        // alone.
        if (robot.configuredWebcams != null) { // if webcam(s) are configured in
            VisionPortalWebcamConfiguration.ConfiguredWebcam frontWebcamConfiguration =
                    robot.configuredWebcams.get(RobotConstantsIntoTheDeep.InternalWebcamId.FRONT_WEBCAM);
            if (frontWebcamConfiguration != null) {
                // The front webcam may configured for raw frames only or for both
                // raw frames and AprilTags.
                EnumMap<RobotConstantsIntoTheDeep.ProcessorIdentifier, Pair<VisionProcessor, Boolean>> assignedProcessors =
                        new EnumMap<>(RobotConstantsIntoTheDeep.ProcessorIdentifier.class);
                VisionProcessor rawFrameProcessor = new RawFrameProcessor();
                assignedProcessors.put(RobotConstantsIntoTheDeep.ProcessorIdentifier.RAW_FRAME, Pair.create(rawFrameProcessor, true));

                if (frontWebcamConfiguration.processorIdentifiers.contains(RobotConstantsIntoTheDeep.ProcessorIdentifier.APRIL_TAG)) {
                    VisionProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
                            // Follow the MultiPortal sample, which only includes setLensIntrinsics
                            .setLensIntrinsics(frontWebcamConfiguration.cameraCalibration.focalLengthX,
                                    frontWebcamConfiguration.cameraCalibration.focalLengthY,
                                    frontWebcamConfiguration.cameraCalibration.principalPointX,
                                    frontWebcamConfiguration.cameraCalibration.principalPointY)
                            .build();

                    assignedProcessors.put(RobotConstantsIntoTheDeep.ProcessorIdentifier.APRIL_TAG, Pair.create(aprilTagProcessor, false));
                }

                VisionPortalWebcam visionPortalWebcam = new VisionPortalWebcam(frontWebcamConfiguration, assignedProcessors);
                frontWebcamConfiguration.setVisionPortalWebcam(visionPortalWebcam);
                if (!visionPortalWebcam.waitForWebcamStart(2000))
                    throw new AutonomousRobotException(TAG, "Unable to start front webcam");

                openWebcams.add(RobotConstantsIntoTheDeep.InternalWebcamId.FRONT_WEBCAM);
            }
        }

        RobotLogCommon.c(TAG, "FTCAuto construction complete");
    }

    public void runRobot() throws Exception {
        // In order for finally() to run we need to have a catch block below.
        // Without it the FTC runtime, which must have an UncaughtExceptionHandler,
        // prevents our finally() from running.
        ActionStatus actionStatus = ActionStatus.NO_ACTIONS_TAKEN; // default
        try {
            // Safety check against the case where the driver hits the small stop
            // button during waitForStart(). We want to make sure that finally()
            // still runs.
            // 12/28/2022 From the FTC SDK documentation: "whether the OpMode is
            // currently active. If this returns false, you should break out of
            // the loop in your runOpMode() method and return to its caller.
            //&& 1/3/2024 Not clear that this condition has ever occurred.
            // But leave the check here and watch the logs.
            if (!linearOpMode.opModeIsActive()) {
                //## Do *not* do this throw new AutonomousRobotException(TAG, "OpMode unexpectedly inactive in runRobot()");
                RobotLog.dd(TAG, "OpMode unexpectedly inactive at the start of runRobot()");
                RobotLogCommon.e(TAG, "OpMode unexpectedly inactive at the start of runRobot()");
                return;
            }

            // Start the Autonomous period expiration timer.
            linearOpMode.resetRuntime();
            autonomousTimer = new AutonomousTimer(linearOpMode);

            RobotLogCommon.i(TAG, "FTCAuto runRobot()");

            // If the StartParameters.xml file contains a non-zero start delay
            // for Autonomous wait here.
            if (autoStartDelay != 0)
                sleepInLoop(autoStartDelay * 1000);

            // Follow the choreography specified in the robot action file.
            List<RobotXMLElement> actions = actionData.actions;
            actionStatus = executeActions(actions);
        } finally {
            //&& 1/3/2024 Experiments (such as letting Autonomous time out)
            // have shown that our finally() block is not executed when the
            // OpMode goes inactive. But leave this check here and watch the
            // logs.
            if (!linearOpMode.opModeIsActive()) {
                RobotLog.dd(TAG, "FTCAuto OpMode not active in finally block");
                RobotLogCommon.i(TAG, "FTCAuto OpMode not active in finally block");
            } else {
                RobotLog.dd(TAG, "In FTCAuto finally block");
                RobotLogCommon.i(TAG, "In FTCAuto finally block");

                autonomousTimer.stopAutonomousTimer();

                // Log the result of the topmost set of actions.
                RobotLog.dd(TAG, "Final action status " + actionStatus);

                if (robot.configuredWebcams != null) { // if webcam(s) are configured in
                    RobotLogCommon.i(TAG, "In FTCAuto finally: close webcam(s)");
                    robot.configuredWebcams.forEach((k, v) -> {
                        if (v != null && v.getVisionPortalWebcam() != null)
                            v.getVisionPortalWebcam().finalShutdown();
                    });
                }
            }
        }

        RobotLogCommon.i(TAG, "Exiting FTCAuto");
        linearOpMode.telemetry.addData("FTCAuto", "COMPLETE");
        linearOpMode.telemetry.update();
    }

    //===============================================================================================
    //===============================================================================================

    private ActionStatus executeActions(List<RobotXMLElement> pActions) throws Exception {
        for (RobotXMLElement action : pActions) {
            if (!linearOpMode.opModeIsActive()) {
                RobotLog.dd(TAG, "OpMode went inactive in the main Autonomous loop");
                RobotLogCommon.d(TAG, "OpMode went inactive in the main Autonomous loop");
                return ActionStatus.OPMODE_INACTIVE;
            }

            // Check the Autonomous timer.
            if (FTCAuto.autonomousTimer.autoTimerIsExpired()) {
                RobotLog.dd(TAG, "Autonomous panic stop triggered during the main Autonomous loop");
                RobotLogCommon.d(TAG, "Autonomous panic stop triggered during the main Autonomous loop");
                return ActionStatus.OPMODE_TIMEOUT;
            }

            ActionStatus actionStatus = executeAction(action);
            if (actionStatus == ActionStatus.ACTION_STOP || actionStatus == ActionStatus.ACTION_STOP_ALL)
                return actionStatus;
        }

        return ActionStatus.ALL_ACTIONS_COMPLETE;
    }

    // Using the XML elements and attributes from the configuration file
    // RobotAction.xml, execute a single action.
    // Note that executeAction may return a value other than ActionStatus.ACTION_COMPLETE
    // as a signal to stop or short-circuit the current set of actions.
    @SuppressLint("DefaultLocale")
    private ActionStatus executeAction(RobotXMLElement pAction) throws Exception {
        // Set up XPath access to the current action.
        XPathAccess actionXPath = new XPathAccess(pAction);
        String actionName = pAction.getRobotXMLElementName().toUpperCase();
        RobotLogCommon.d(TAG, "Executing FTCAuto action " + actionName);

        switch (actionName) {
            case "RUN_BLUE_A3": {
                runBlueA3();
                break;
            }
            case "RUN_BLUE_A4": {
                runBlueA4();
                break;
            }
            case "RUN_RED_F3": {
                runRedF3();
                break;
            }
            case "RUN_RED_F4": {
                runRedF4();
                break;
            }


            case "START_WEBCAM": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsIntoTheDeep.InternalWebcamId webcamId =
                        RobotConstantsIntoTheDeep.InternalWebcamId.valueOf(webcamIdString);

                if (!openWebcams.add(webcamId))
                    throw new AutonomousRobotException(TAG, "Attempt to start webcam " + webcamId + " but it is already open");

                VisionPortalWebcamConfiguration.ConfiguredWebcam configuredWebcam =
                        robot.configuredWebcams.get(webcamId);
                if (configuredWebcam == null)
                    throw new AutonomousRobotException(TAG, "Attempt to start a webcam that is not in the configuration " + webcamId);

                // The <START_WEBCAM> in RobotAction.xml may contain a single
                // <processor> element or a <processor_set> element with
                // multiple <processor> children.
                ArrayList<Pair<RobotConstantsIntoTheDeep.ProcessorIdentifier, Boolean>> requestedProcessors;
                String processorIdString = actionXPath.getText("processor", "NPOS").toUpperCase();
                if (!processorIdString.equals("NPOS")) { // single processor id
                    RobotConstantsIntoTheDeep.ProcessorIdentifier processorId = RobotConstantsIntoTheDeep.ProcessorIdentifier.valueOf(processorIdString);
                    requestedProcessors = new ArrayList<Pair<RobotConstantsIntoTheDeep.ProcessorIdentifier, Boolean>>() {{
                        add(Pair.create(processorId, true)); // default to enable on webcam start
                    }};
                } else // multiple processors, i.e. a <processor_set>.
                    requestedProcessors = RobotActionXML.getStartWebcamProcessors(pAction);

                // The collection requestedProcessors contains the ids of one or
                // more processors to be assigned to the webcam. Check that each
                // processor id is present in the webcam's <processor_set> in
                // RobotConfig.xml and create the actual VisionProcessor objects.
                EnumMap<RobotConstantsIntoTheDeep.ProcessorIdentifier, Pair<VisionProcessor, Boolean>> assignedProcessors =
                        new EnumMap<>(RobotConstantsIntoTheDeep.ProcessorIdentifier.class);
                ArrayList<RobotConstantsIntoTheDeep.ProcessorIdentifier> processorIdentifiers = configuredWebcam.processorIdentifiers;
                for (Pair<RobotConstantsIntoTheDeep.ProcessorIdentifier, Boolean> entry : requestedProcessors) {
                    if (!processorIdentifiers.contains(entry.first))
                        throw new AutonomousRobotException(TAG, "Assigned processor with id " + entry.first + " is not in the configuration for webcam " + configuredWebcam.internalWebcamId);

                    switch (entry.first) {
                        case RAW_FRAME: {
                            RawFrameProcessor rawFrameProcessor = new RawFrameProcessor();
                            assignedProcessors.put(RobotConstantsIntoTheDeep.ProcessorIdentifier.RAW_FRAME,
                                    Pair.create(rawFrameProcessor, entry.second));
                            break;
                        }
                        case APRIL_TAG: {
                            AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
                                    // Follow the MultiPortal sample, which only includes setLensIntrinsics
                                    .setLensIntrinsics(configuredWebcam.cameraCalibration.focalLengthX,
                                            configuredWebcam.cameraCalibration.focalLengthY,
                                            configuredWebcam.cameraCalibration.principalPointX,
                                            configuredWebcam.cameraCalibration.principalPointY)
                                    .build();

                            assignedProcessors.put(RobotConstantsIntoTheDeep.ProcessorIdentifier.APRIL_TAG,
                                    Pair.create(aprilTagProcessor, entry.second));
                            break;
                        }

                        default:
                            throw new AutonomousRobotException(TAG, "Invalid processor id " + entry.first);
                    }
                }

                // Now actually create the VisionPortalWebcam with the assigned
                // (but not yet enabled) processors.
                VisionPortalWebcam visionPortalWebcam = new VisionPortalWebcam(configuredWebcam, assignedProcessors);
                configuredWebcam.setVisionPortalWebcam(visionPortalWebcam);
                openWebcams.add(webcamId);

                break;
            }

            case "WAIT_FOR_WEBCAM_START": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsIntoTheDeep.InternalWebcamId webcamId =
                        RobotConstantsIntoTheDeep.InternalWebcamId.valueOf(webcamIdString);

                if (!openWebcams.contains(webcamId))
                    throw new AutonomousRobotException(TAG, "Attempt to wait for the startup of webcam " + webcamId + " but it is not open");

                VisionPortalWebcamConfiguration.ConfiguredWebcam configuredWebcam =
                        robot.configuredWebcams.get(webcamId);
                if (configuredWebcam == null)
                    throw new AutonomousRobotException(TAG, "Attempt to start a webcam that is not in the configuration " + webcamId);

                int timeout = actionXPath.getRequiredInt("timeout_ms");
                if (!configuredWebcam.getVisionPortalWebcam().waitForWebcamStart(timeout)) {
                    return ActionStatus.ACTION_STOP_ALL; // no webcam, just give up
                }

                break;
            }

            case "STOP_WEBCAM": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsIntoTheDeep.InternalWebcamId webcamId =
                        RobotConstantsIntoTheDeep.InternalWebcamId.valueOf(webcamIdString);

                if (!openWebcams.contains(webcamId))
                    throw new AutonomousRobotException(TAG, "Attempt to close webcam " + webcamId + " but it is not open");

                VisionPortalWebcamConfiguration.ConfiguredWebcam configuredWebcam =
                        robot.configuredWebcams.get(webcamId);
                if (configuredWebcam == null)
                    throw new AutonomousRobotException(TAG, "Attempt to start a webcam that is not in the configuration " + webcamId);

                configuredWebcam.getVisionPortalWebcam().finalShutdown();
                configuredWebcam.setVisionPortalWebcam(null);
                openWebcams.remove(webcamId);
                RobotLogCommon.d(TAG, "Stopped webcam " + webcamIdString);
                break;
            }

            case "STOP_WEBCAM_STREAMING": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsIntoTheDeep.InternalWebcamId webcamId =
                        RobotConstantsIntoTheDeep.InternalWebcamId.valueOf(webcamIdString);
                VisionPortalWebcamConfiguration.ConfiguredWebcam webcam =
                        Objects.requireNonNull(robot.configuredWebcams.get(webcamId), TAG + " Webcam " + webcamId + " is not in the current configuration");

                if (!openWebcams.contains(webcamId))
                    throw new AutonomousRobotException(TAG, "Attempt to stop streaming webcam " + webcamId + " but it is not open");

                webcam.getVisionPortalWebcam().stopStreaming();
                RobotLogCommon.d(TAG, "Stopped streaming webcam " + webcamIdString);
                break;
            }

            case "RESUME_WEBCAM_STREAMING": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsIntoTheDeep.InternalWebcamId webcamId =
                        RobotConstantsIntoTheDeep.InternalWebcamId.valueOf(webcamIdString);
                VisionPortalWebcamConfiguration.ConfiguredWebcam webcam =
                        Objects.requireNonNull(robot.configuredWebcams.get(webcamId), TAG + " Webcam " + webcamId + " is not in the current configuration");

                if (!openWebcams.contains(webcamId))
                    throw new AutonomousRobotException(TAG, "Attempt to resume streaming webcam " + webcamId + " but it is not open");

                int timeout = actionXPath.getRequiredInt("timeout_ms");
                webcam.getVisionPortalWebcam().resumeStreaming(timeout);
                RobotLogCommon.d(TAG, "Resumed streaming webcam " + webcamIdString);
                break;
            }

            case "ENABLE_PROCESSOR": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsIntoTheDeep.InternalWebcamId webcamId =
                        RobotConstantsIntoTheDeep.InternalWebcamId.valueOf(webcamIdString);
                VisionPortalWebcamConfiguration.ConfiguredWebcam webcam =
                        Objects.requireNonNull(robot.configuredWebcams.get(webcamId), TAG + " Webcam " + webcamId + " is not in the current configuration");

                if (!openWebcams.contains(webcamId))
                    throw new AutonomousRobotException(TAG, "Attempt to enable processor on webcam " + webcamId + " but it is not open");

                String processorIdString = actionXPath.getRequiredText("processor").toUpperCase();
                RobotConstantsIntoTheDeep.ProcessorIdentifier processorId = RobotConstantsIntoTheDeep.ProcessorIdentifier.valueOf(processorIdString);

                webcam.getVisionPortalWebcam().enableProcessor(processorId);
                RobotLogCommon.d(TAG, "Enabled processor on webcam " + webcamIdString);
                break;
            }

            case "DISABLE_PROCESSOR": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsIntoTheDeep.InternalWebcamId webcamId =
                        RobotConstantsIntoTheDeep.InternalWebcamId.valueOf(webcamIdString);
                VisionPortalWebcamConfiguration.ConfiguredWebcam webcam =
                        Objects.requireNonNull(robot.configuredWebcams.get(webcamId), TAG + " Webcam " + webcamId + " is not in the current configuration");

                if (!openWebcams.contains(webcamId))
                    throw new AutonomousRobotException(TAG, "Attempt to disable processor on webcam " + webcamId + " but it is not open");

                String processorIdString = actionXPath.getRequiredText("processor").toUpperCase();
                RobotConstantsIntoTheDeep.ProcessorIdentifier processorId = RobotConstantsIntoTheDeep.ProcessorIdentifier.valueOf(processorIdString);

                webcam.getVisionPortalWebcam().disableProcessor(processorId);
                RobotLogCommon.d(TAG, "Disabled processor on webcam " + webcamIdString);
                break;
            }

            // For testing, get a frame from a webcam managed by the VisionPortal
            // API and write it out to a file. Assume that the webcam has already
            // been started.
            case "TAKE_PICTURE_WEBCAM": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsIntoTheDeep.InternalWebcamId webcamId =
                        RobotConstantsIntoTheDeep.InternalWebcamId.valueOf(webcamIdString);
                VisionPortalWebcamConfiguration.ConfiguredWebcam webcam =
                        Objects.requireNonNull(robot.configuredWebcams.get(webcamId), TAG + " Webcam " + webcamId + " is not in the current configuration");

                if (!openWebcams.contains(webcamId))
                    throw new AutonomousRobotException(TAG, "Attempt to take picture on webcam " + webcamId + " but it is not open");

                RawFrameProcessor rawFrameProcessor =
                        (RawFrameProcessor) webcam.getVisionPortalWebcam().getEnabledProcessor(RobotConstantsIntoTheDeep.ProcessorIdentifier.RAW_FRAME);
                if (rawFrameProcessor == null)
                    throw new AutonomousRobotException(TAG, "The RAW_FRAME processor is not active");

                Pair<Mat, Date> image = rawFrameProcessor.getImage();
                if (image == null) {
                    RobotLogCommon.d(TAG, "Unable to get image from " + webcamIdString);
                    linearOpMode.telemetry.addData("Take picture:", "unable to get image from " + webcamIdString);
                    linearOpMode.telemetry.update();
                } else {
                    RobotLogCommon.d(TAG, "Took a picture with " + webcamIdString);
                    String fileDate = TimeStamp.getDateTimeStamp(image.second);
                    String outputFilenamePreamble = workingDirectory + RobotConstants.IMAGE_DIR + webcamIdString + "_" + fileDate;

                    String imageFilename = outputFilenamePreamble + "_IMG.png";
                    RobotLogCommon.d(TAG, "Writing image " + imageFilename);
                    Imgcodecs.imwrite(imageFilename, image.first);

                    RobotLogCommon.d(TAG, "Image width " + image.first.cols() + ", height " + image.first.rows());
                    linearOpMode.telemetry.addData("Take picture with webcam:", "successful");
                    linearOpMode.telemetry.update();
                }

                break;
            }

            // For testing: look for all AprilTags in a loop for 10 seconds.
            case "FIND_ALL_APRIL_TAGS": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsIntoTheDeep.InternalWebcamId webcamId =
                        RobotConstantsIntoTheDeep.InternalWebcamId.valueOf(webcamIdString);

                VisionPortalWebcamConfiguration.ConfiguredWebcam webcam =
                        Objects.requireNonNull(robot.configuredWebcams.get(webcamId), TAG + " Webcam " + webcamId + " is not in the current configuration");

                if (!openWebcams.contains(webcamId))
                    throw new AutonomousRobotException(TAG, "Attempt to find an AprilTag on webcam " + webcamId + " but it is not open");

                VisionProcessor aprilTagProcessor =
                        webcam.getVisionPortalWebcam().getEnabledProcessor(RobotConstantsIntoTheDeep.ProcessorIdentifier.APRIL_TAG);
                if (aprilTagProcessor == null)
                    throw new AutonomousRobotException(TAG, "The APRIL_TAG processor is not active");

                ElapsedTime aprilTagTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                aprilTagTimer.reset();
                List<AprilTagDetection> currentDetections;
                boolean aprilTagDetected;
                while (linearOpMode.opModeIsActive() && !autonomousTimer.autoTimerIsExpired() &&
                        aprilTagTimer.time() < 10000) {
                    aprilTagDetected = false;
                    currentDetections = AprilTagAccess.getAprilTagData((AprilTagProcessor) aprilTagProcessor, 500);
                    for (AprilTagDetection detection : currentDetections) {
                        if (detection.metadata != null) {
                            aprilTagDetected = true;
                            break; // don't look any further.
                        }
                    }

                    if (!aprilTagDetected) {
                        linearOpMode.telemetry.addLine("No AprilTags found within 500ms");
                        linearOpMode.telemetry.update();
                        RobotLogCommon.d(TAG, "No AprilTags found within 500ms");
                    } else
                        telemetryAprilTag(currentDetections);

                    sleep(250); // be careful - small sleep values flood the log
                }

                break;
            }

            // Look for a specific AprilTag
            case "FIND_APRIL_TAG": {
                String tagIdString = actionXPath.getRequiredText("tag_id").toUpperCase();
                AprilTagUtils.AprilTagId targetTagId = AprilTagUtils.AprilTagId.valueOf(tagIdString);
                detectAprilTag(targetTagId, actionXPath);
                break;
            }

            //## For testing.
            case "LAUNCH_DRONE": {
                robot.droneLauncherServo.hold();
                sleep(500);

                robot.droneLauncherServo.launch();
                sleep(500);
                break;
            }

            //## For testing, click count and velocity are arbitrary.
            case "RUN_INTAKE": {
                RobotLogCommon.d(TAG, "Running the INTAKE");
                robot.intakeMotion.resetAndMoveSingleMotor(robot.intake.deliver_front, 0.25, SingleMotorMotion.MotorAction.MOVE_AND_STOP);
                break;
            }

            //## For testing, click count is arbitrary.
            case "RUN_ELEVATOR": {
                RobotLogCommon.d(TAG, "Running the ELEVATOR");
                robot.elevatorMotion.moveElevator(robot.elevator.level_2, robot.elevator.velocity, ElevatorMotion.ElevatorAction.MOVE_AND_STOP);
                break;
            }

            case "SLEEP": { // I want sleep :)
                int sleepMs = actionXPath.getRequiredInt("ms");
                sleepInLoop(sleepMs);
                break;
            }

            // In testing this gives us a way to short-circuit a set
            //  of actions without commenting out any XML.
            case "STOP": {
                sleep(500);
                return ActionStatus.ACTION_STOP;
            }

            // In testing this gives us a way to short-circuit all
            // actions without commenting out any XML.
            case "STOP_ALL": {
                sleep(500);
                return ActionStatus.ACTION_STOP_ALL;
            }

            default: {
                throw new AutonomousRobotException(TAG, "No support for the action " + actionName);
            }
        }

        // Action completed normally.
        return ActionStatus.ACTION_COMPLETE;
    }

    private void runBlueA3() {
        //**TODO fill in runBlueA3;
    }

    private void runBlueA4() {
        //**TODO fill in runBlueA4;
    }

    // The field MecanumDrive roadrunnerDrive has already been set with the
    // starting pose.
    private void runRedF3() {
        //**TODO fill in runRedF3;
    }

    private void runRedF4() {
        // The field drive.pose is not actually used here.
        Action hangSpecimen = roadrunnerDrive.actionBuilder(roadrunnerDrive.pose)
                .waitSeconds(2)
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        TrajectoryActionCollection.buildTrajectoryAction(roadrunnerDrive, roadrunnerDrive.pose, TrajectoryActionCollection.TrajectoryActionId.RED_F4_TO_SUBMERSIBLE),
                        hangSpecimen,
                        new LinkedTrajectoryAction(roadrunnerDrive, TrajectoryActionCollection.TrajectoryActionId.RED_F4_TO_SAMPLE_1)
                )
        );
    }

    // Sleeps but also tests if the OpMode is still active.
    // Telemetry should keep the FTC runtime from shutting us down due to inactivity.
    private void sleepInLoop(int pMilliseconds) {
        RobotLogCommon.d(TAG, "Sleep for " + pMilliseconds + " milliseconds");

        int numSleeps = pMilliseconds / 100;
        int sleepRemainder = pMilliseconds % 100;
        for (int i = 0; i < numSleeps; i++) {
            if (!linearOpMode.opModeIsActive() || autonomousTimer.autoTimerIsExpired())
                return;
            linearOpMode.sleep(100);
        }

        if (linearOpMode.opModeIsActive() && !autonomousTimer.autoTimerIsExpired()
                && sleepRemainder != 0)
            linearOpMode.sleep(sleepRemainder);
    }

    // Copied from the sample ConceptAprilTag and slightly modified.
    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag(List<AprilTagDetection> pCurrentDetections) {

        linearOpMode.telemetry.addData("# AprilTags Detected", pCurrentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : pCurrentDetections) {
            if (detection.metadata != null) {
                String detectionId = String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name);
                String XYZ = String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z);
                String PRY = String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw);
                String RBE = String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation);

                linearOpMode.telemetry.addLine(detectionId);
                linearOpMode.telemetry.addLine(XYZ);
                linearOpMode.telemetry.addLine(PRY);
                linearOpMode.telemetry.addLine(RBE);

                RobotLogCommon.d(TAG, detectionId);
                RobotLogCommon.d(TAG, XYZ);
                RobotLogCommon.d(TAG, PRY);
                RobotLogCommon.d(TAG, RBE);
            } else {
                String unknownId = String.format("\n==== (ID %d) Unknown", detection.id);
                String center = String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y);

                linearOpMode.telemetry.addLine(unknownId);
                linearOpMode.telemetry.addLine(center);
                RobotLogCommon.d(TAG, unknownId);
                RobotLogCommon.d(TAG, center);
            }
        }   // end for() loop

        // Add "key" information to telemetry
        linearOpMode.telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        linearOpMode.telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        linearOpMode.telemetry.addLine("RBE = Range, Bearing & Elevation");
        linearOpMode.telemetry.update();

        RobotLogCommon.d(TAG, "\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        RobotLogCommon.d(TAG, "PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        RobotLogCommon.d(TAG, "RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

    @SuppressLint("DefaultLocale")
    private AprilTagDetectionData detectAprilTag(AprilTagUtils.AprilTagId pTargetTagId, XPathAccess pActionXPath) throws XPathExpressionException {
        String webcamIdString = pActionXPath.getRequiredText("internal_webcam_id").toUpperCase();
        RobotConstantsIntoTheDeep.InternalWebcamId webcamId =
                RobotConstantsIntoTheDeep.InternalWebcamId.valueOf(webcamIdString);

        VisionPortalWebcamConfiguration.ConfiguredWebcam webcam =
                Objects.requireNonNull(robot.configuredWebcams.get(webcamId), TAG + " Webcam " + webcamId + " is not in the current configuration");

        if (!openWebcams.contains(webcamId))
            throw new AutonomousRobotException(TAG, "Attempt to find an AprilTag on webcam " + webcamId + " but it is not open");

        VisionProcessor aprilTagProcessor =
                webcam.getVisionPortalWebcam().getEnabledProcessor(RobotConstantsIntoTheDeep.ProcessorIdentifier.APRIL_TAG);
        if (aprilTagProcessor == null)
            throw new AutonomousRobotException(TAG, "The APRIL_TAG processor is not active");

        int timeout = pActionXPath.getRequiredInt("timeout_ms");
        List<AprilTagDetection> currentDetections = AprilTagAccess.getAprilTagData((AprilTagProcessor) aprilTagProcessor, timeout);
        AprilTagDetection targetDetection = null;
        AprilTagDetection backupDetection = null;
        double smallestBackupAngle = 360.0; // impossibly high

        // Step through the list of detected tags and look for a matching tag.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == pTargetTagId.getNumericId()) {
                    targetDetection = detection;
                    break; // don't look any further.
                } else {
                    if (Math.abs(detection.ftcPose.bearing) < smallestBackupAngle) {
                        smallestBackupAngle = detection.ftcPose.bearing;
                        backupDetection = detection;
                    }
                }
            }
        }

        // If we have found the target AprilTag return it now.
        if (targetDetection != null) {
            String targetTagString = "Found target AprilTag " + String.format("Id %d (%s)", targetDetection.id, targetDetection.metadata.name);
            String range = "Range " + String.format("%5.1f inches", targetDetection.ftcPose.range);
            String bearing = "Bearing " + String.format("%3.0f degrees", targetDetection.ftcPose.bearing);
            String yaw = "Yaw " + String.format("%3.0f degrees", targetDetection.ftcPose.yaw);

            linearOpMode.telemetry.addLine(targetTagString);
            linearOpMode.telemetry.update();

            RobotLogCommon.d(TAG, targetTagString);
            RobotLogCommon.d(TAG, range);
            RobotLogCommon.d(TAG, bearing);
            RobotLogCommon.d(TAG, yaw);
            return new AprilTagDetectionData(webcamId, pTargetTagId, targetDetection);
        }

        // If we have not found the target target, see if we've found one of
        // the other AprilTags to use as a backup.
        if (backupDetection == null) {
            linearOpMode.telemetry.addLine("No AprilTags found within " + timeout + "ms");
            linearOpMode.telemetry.update();
            RobotLogCommon.d(TAG, "No AprilTags found within " + timeout + "ms");
            return new AprilTagDetectionData(webcamId, pTargetTagId, null);
        }

        // Found a backup detection.
        AprilTagUtils.AprilTagId backupTagId = getEnumValue(backupDetection.id);
        String backupTagString = "Found backup AprilTag " + String.format("Id %d (%s)", backupDetection.id, backupDetection.metadata.name);
        String range = "Range " + String.format("%5.1f inches", backupDetection.ftcPose.range);
        String bearing = "Bearing " + String.format("%3.0f degrees", backupDetection.ftcPose.bearing);
        String yaw = "Yaw " + String.format("%3.0f degrees", backupDetection.ftcPose.yaw);

        linearOpMode.telemetry.addLine(backupTagString);
        linearOpMode.telemetry.update();

        RobotLogCommon.d(TAG, backupTagString);
        RobotLogCommon.d(TAG, range);
        RobotLogCommon.d(TAG, bearing);
        RobotLogCommon.d(TAG, yaw);
        return new AprilTagDetectionData(webcamId, backupTagId, backupDetection);
    }

    private static class AprilTagDetectionData {
        public final RobotConstantsIntoTheDeep.InternalWebcamId webcamId;
        public final AprilTagUtils.AprilTagId aprilTagId;
        public final AprilTagDetection ftcDetectionData;

        public AprilTagDetectionData(RobotConstantsIntoTheDeep.InternalWebcamId pWebcamId,
                                     AprilTagUtils.AprilTagId pAprilTagId,
                                     AprilTagDetection pFtcDetectionData) {
            webcamId = pWebcamId;
            aprilTagId = pAprilTagId;
            ftcDetectionData = pFtcDetectionData;
        }
    }

}

