package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsIntoTheDeep;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.firstinspires.ftc.teamcode.robot.device.camera.VisionPortalWebcamConfiguration;
import org.firstinspires.ftc.teamcode.robot.device.motor.Elevator;
import org.firstinspires.ftc.teamcode.robot.device.motor.ElevatorMotion;
import org.firstinspires.ftc.teamcode.robot.device.motor.Intake;
import org.firstinspires.ftc.teamcode.robot.device.motor.SingleMotorMotion;
import org.firstinspires.ftc.teamcode.robot.device.motor.TeleOpDriveTrain;
import org.firstinspires.ftc.teamcode.robot.device.servo.DroneLauncherServo;
import org.firstinspires.ftc.teamcode.xml.RobotConfigXML;
import org.firstinspires.ftc.teamcode.xml.StartParameters;
import org.firstinspires.ftc.teamcode.xml.StartParametersXML;
import org.xml.sax.SAXException;

import java.io.IOException;
import java.text.DecimalFormat;
import java.util.EnumMap;
import java.util.Optional;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPathExpressionException;

public class FTCRobot {

    private static final String TAG = FTCRobot.class.getSimpleName();

    // XML element names in RobotConfig.xml
    public static final String DRIVE_TRAIN_ELEMENT_NAME = "DRIVE_TRAIN";
    public static final String TELEOP_SETTINGS_ELEMENT_NAME = "TELEOP_SETTINGS";
    public static final String ELEVATOR_ELEMENT_NAME = "ELEVATOR";
    public static final String INTAKE_ELEMENT_NAME = "INTAKE";
    public static final String DRONE_LAUNCHER_ELEMENT_NAME = "DRONE_LAUNCHER";
    public static final String VISION_PORTAL_WEBCAM_ELEMENT_NAME = "VISION_PORTAL_WEBCAM";

    // All motors on the robot for this year's game.
    public enum MotorId {
        LEFT_FRONT_DRIVE, RIGHT_FRONT_DRIVE, LEFT_BACK_DRIVE, RIGHT_BACK_DRIVE,
        INTAKE, //**TODO for testing XML and SingleMotorCore
        LEFT_ELEVATOR, RIGHT_ELEVATOR, //**TODO for testing XML and MultiMotorCore
        MOTOR_ID_NPOS // for error checking
    }

    public enum ServoId {
        DRONE_LAUNCHER //**TODO for testing XML
    }

    private final HardwareMap hardwareMap;
    public final StartParameters startParameters;

    // Needs to be static so that Roadrunner MecanumDrive can see it.
    // This makes for a really unfortunate dependency - FTCRobot must
    // be constructed before MecanumDrive; otherwise driveTrainDeviceNames
    // will be null.
    private static DriveTrainDeviceNames driveTrainDeviceNames;
    public final TeleOpDriveTrain teleOpDriveTrain;
    public final boolean roadrunnerDriveTrainInConfiguration;
    public final TeleOpSettings teleOpSettings;
    public final Elevator elevator;
    public final ElevatorMotion elevatorMotion;
    public final Intake intake;
    public final SingleMotorMotion intakeMotion;
    public final DroneLauncherServo droneLauncherServo;

    // Instantiate here in FTCRobot so that these objects
    // can be shared between TeleOp and FTCAuto when it is
    // embedded within TeleOp.
    public final EnumMap<RobotConstantsIntoTheDeep.InternalWebcamId, VisionPortalWebcamConfiguration.ConfiguredWebcam> configuredWebcams;

    public FTCRobot(LinearOpMode pLinearOpMode, RobotConstants.RunType pRunType) throws InterruptedException {
        hardwareMap = pLinearOpMode.hardwareMap;

        RobotLogCommon.c(TAG, "FTCRobot constructor");

        //!! WARNING
        // From the FTC example ConceptMotorBulkRead.java
        // Important Step 1:  Make sure you use DcMotorEx when you instantiate your motors. [See DriveTrainCore.java]
        // Important Step 2: Get access to a list of Expansion Hub Modules to enable changing caching methods.

        //!! 11/16/21 commented out AND NEEDS TO REMAIN OUT because of unpredictable
        // behavior with RUN_TO_POSITION, e.g. immediate reporting of completion of RTP
        // with ending encoder counts of 0 - even though the movement actually took
        // place.
        /*
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        // Important Step 3: Set all Expansion hubs to use the AUTO Bulk Caching mode.
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        */

        String xmlDirectory = WorkingDirectory.getWorkingDirectory() + RobotConstants.XML_DIR;

        // Get the hardware configuration parameters from RobotConfig XXX.xml.
        try {
            // Get the startup parameters (including the exact file name of
            // RobotConfig XXX.xml).
            // Get the configurable startup parameters.
            StartParametersXML startParametersXML = new StartParametersXML(xmlDirectory);
            startParameters = startParametersXML.getStartParameters();
            RobotLogCommon.c(TAG, "Configuring the robot from " + startParameters.robotConfigFilename);

            RobotConfigXML configXML = new RobotConfigXML(startParameters.robotConfigFilename);
            XPathAccess configXPath;

            // If a drive train is defined and we're running an Autonomous OpMode,
            // which implies that Roadrunner is controlling the drive train, then
            // defer the initialization of the drive train to FTCAuto. Roadrunner
            // needs to know the starting pose of the robot - this is only
            // available after the OpMode is selected.
            configXPath = configXML.getPath(DRIVE_TRAIN_ELEMENT_NAME);
            String driveTrainYesNo = configXPath.getRequiredTextInRange("@configured", configXPath.validRange("yes", "no"));
            RobotLogCommon.c(TAG, "Drive train configuration option: " + driveTrainYesNo);
            if (driveTrainYesNo.equals("yes")) {
                // If we're running a TeleOp OpMode, get the TeleOp settings.
                if (pRunType == RobotConstants.RunType.TELEOP || pRunType == RobotConstants.RunType.TELEOP_VISION_PREVIEW) {
                    teleOpDriveTrain = new TeleOpDriveTrain(hardwareMap, configXPath, DRIVE_TRAIN_ELEMENT_NAME);
                    roadrunnerDriveTrainInConfiguration = false;
                    driveTrainDeviceNames = null;

                    XPathAccess teleOpSettingsXPath = configXML.getPath(TELEOP_SETTINGS_ELEMENT_NAME);
                    String logging_level = teleOpSettingsXPath.getRequiredTextInRange("log_level", teleOpSettingsXPath.validRange("d", "v", "vv", "off"));
                    double driveTrainPowerHigh = teleOpSettingsXPath.getRequiredDouble("drive_train_power/high");
                    double driveTrainPowerLow = teleOpSettingsXPath.getRequiredDouble("drive_train_power/low");

                    teleOpSettings = new TeleOpSettings(logging_level, driveTrainPowerHigh, driveTrainPowerLow);
                    RobotLogCommon.c(TAG, "TeleOp configuration: log level " + teleOpSettings.logLevel);
                    RobotLogCommon.setMostDetailedLogLevel(teleOpSettings.logLevel);
                } else {
                    roadrunnerDriveTrainInConfiguration = true;

                    // Roadrunner is in the configuration so provide the drive train
                    // device names from RobotConfig.xml.
                    driveTrainDeviceNames = new DriveTrainDeviceNames(configXPath, DRIVE_TRAIN_ELEMENT_NAME,
                            MotorId.LEFT_FRONT_DRIVE, MotorId.RIGHT_FRONT_DRIVE,
                            MotorId.LEFT_BACK_DRIVE, MotorId.RIGHT_BACK_DRIVE);
                    teleOpDriveTrain = null;
                    teleOpSettings = null;
                }
            } else {
                roadrunnerDriveTrainInConfiguration = false;
                driveTrainDeviceNames = null;
                teleOpDriveTrain = null;
                teleOpSettings = null;
            }

            // Get the configuration for the dual-motor elevator.
            configXPath = configXML.getPath(ELEVATOR_ELEMENT_NAME);
            String elevatorInConfiguration = configXPath.getRequiredTextInRange("@configured", configXPath.validRange("yes", "no"));
            if (elevatorInConfiguration.equals("yes")) {
                elevator = new Elevator(hardwareMap, configXPath, ELEVATOR_ELEMENT_NAME);
                elevatorMotion = new ElevatorMotion(pLinearOpMode, elevator);
            } else {
                elevator = null;
                elevatorMotion = null;
            }

            // Get the configuration for pixel intake/outtake.
            configXPath = configXML.getPath(INTAKE_ELEMENT_NAME);
            String intakeInConfiguration = configXPath.getRequiredTextInRange("@configured", configXPath.validRange("yes", "no"));
            if (intakeInConfiguration.equals("yes")) {
                intake = new Intake(hardwareMap, configXPath, INTAKE_ELEMENT_NAME);
                intakeMotion = new SingleMotorMotion(pLinearOpMode, intake);
            } else {
                intake = null;
                intakeMotion = null;
            }

            // Get the configuration for the drone launcher servo.
            configXPath = configXML.getPath(DRONE_LAUNCHER_ELEMENT_NAME);
            String launcherConfiguration = configXPath.getRequiredTextInRange("@configured", configXPath.validRange("yes", "no"));
            if (launcherConfiguration.equals("yes")) {
                droneLauncherServo = new DroneLauncherServo(hardwareMap, configXPath, DRONE_LAUNCHER_ELEMENT_NAME, ServoId.DRONE_LAUNCHER);
            } else {
                droneLauncherServo = null;
            }

            // In a competition the webcam(s) would be configured in and
            // used in Autonomous but not in TeleOp so we can't just check
            // the configuration file.
            if (pRunType == RobotConstants.RunType.TELEOP) {
                configuredWebcams = null;
            } else {
                // Any configured VisionPortal webcams?
                EnumMap<RobotConstantsIntoTheDeep.InternalWebcamId, VisionPortalWebcamConfiguration.ConfiguredWebcam> configuredWebcamsLocal;
                configXPath = configXML.getPath(VISION_PORTAL_WEBCAM_ELEMENT_NAME);
                String webcamYesNo = configXPath.getRequiredTextInRange("@configured", configXPath.validRange("yes", "no"));
                RobotLogCommon.c(TAG, "VisionPortal webcam configuration option: " + webcamYesNo);

                if (webcamYesNo.equals("yes")) {
                    configuredWebcamsLocal = configXML.getConfiguredWebcams();
                    RobotLogCommon.d(TAG, "Number of webcams configured " + configuredWebcamsLocal.size());
                    if (configuredWebcamsLocal.size() > 2)
                        throw new AutonomousRobotException(TAG, "CenterStage season: only two webcams at most are supported");

                    matchHardwareWebcamsWithConfiguredWebcams(configuredWebcamsLocal);
                } else
                    configuredWebcamsLocal = null;

                configuredWebcams = configuredWebcamsLocal; // needed to preserve "final"
            }

        } catch (ParserConfigurationException | SAXException | XPathExpressionException |
                 IOException ex) {
            String eMessage = ex.getMessage() == null ? "**no error message**" : ex.getMessage();
            throw new AutonomousRobotException(TAG, "IOException " + eMessage);
        }
    }

    public static DriveTrainDeviceNames getDriveTrainDeviceNames() {
        return driveTrainDeviceNames;
    }

    // The FTC SDK uses a string such as "Webcam 1" to connect
    // a webcam via the HardwareMap and to create a WebcamName
    // object. Use the webcam's serial number in its WebcamName
    // object to associate the webcam with its counterpart in
    // RobotConfig.xml.
    private void matchHardwareWebcamsWithConfiguredWebcams(EnumMap<RobotConstantsIntoTheDeep.InternalWebcamId, VisionPortalWebcamConfiguration.ConfiguredWebcam> pConfiguredWebcams) {
        String webcamId;
        for (int i = 1; i <= pConfiguredWebcams.size(); i++) {
            webcamId = "Webcam " + new DecimalFormat("0").format(i);
            WebcamName webcamName = hardwareMap.get(WebcamName.class, webcamId);
            if (!webcamName.isWebcam() || !webcamName.isAttached())
                throw new AutonomousRobotException(TAG, "Webcam " + webcamId +
                        " is not a webcam or is not attached");

            //RobotLogCommon.d(TAG, "Webcam hardware device " + webcamId +
            //        " is attached to webcam serial number " + webcamName.getSerialNumber());

            Optional<VisionPortalWebcamConfiguration.ConfiguredWebcam> configuredWebcam = pConfiguredWebcams.values().stream()
                    .filter(webcam -> webcam.serialNumber.equals(webcamName.getSerialNumber().toString()))
                    .findFirst();

            if (!configuredWebcam.isPresent())
                throw new AutonomousRobotException(TAG,
                        "No configured webcam for serial number: " + webcamName.getSerialNumber());

            // Now that we have the correct association, add the WebcamName to the
            // configured camera.
            VisionPortalWebcamConfiguration.ConfiguredWebcam configuredWebcamObject = configuredWebcam.get();
            configuredWebcamObject.setWebcamName(webcamName);
            RobotLogCommon.i(TAG, "Webcam hardware device " + webcamId +
                    " is associated by serial number " + webcamName.getSerialNumber() +
                    " with configured webcam " + configuredWebcamObject.internalWebcamId);
        }
    }

    // We need to work around the fact that the standard release of Roadrunner
    // hardcodes the drive train device names in MecanumDrive.java. We put the
    // device names into RobotConfig.xml.
    public static class DriveTrainDeviceNames {
        private static final String TAG = DriveTrainDeviceNames.class.getSimpleName();

        public final EnumMap<FTCRobot.MotorId, String> deviceNames = new EnumMap<>(FTCRobot.MotorId.class);

        public DriveTrainDeviceNames(XPathAccess pConfigXPath, String pDriveTrainElementName, FTCRobot.MotorId... pMotorIds) throws XPathExpressionException {

            // First a sanity check that we're point at the right element in RobotConfig.xml.
            if (!pDriveTrainElementName.equals(FTCRobot.DRIVE_TRAIN_ELEMENT_NAME))
                throw new AutonomousRobotException(TAG, "Missing expected " + FTCRobot.DRIVE_TRAIN_ELEMENT_NAME + " element");

            // Get the configuration from RobotConfig.xml.
            RobotLogCommon.c(TAG, "Defining the device names for the " + FTCRobot.DRIVE_TRAIN_ELEMENT_NAME);

            for (FTCRobot.MotorId motorId: pMotorIds) {
                // The motor element name is the lowercase version of the MotorId.
                String motorDeviceName = pConfigXPath.getRequiredText("multiple_motors/" + motorId.toString().toLowerCase()  + "/@device_name");
                deviceNames.put(motorId, motorDeviceName);
            }
        }
    }

    // Fields captured from RobotConfig.xml.
    public static class TeleOpSettings {
        public final RobotLogCommon.CommonLogLevel logLevel;
        public final double driveTrainPowerHigh, driveTrainPowerLow;

        public TeleOpSettings(String pLogLevel,
                              double pDriveTrainPowerHigh, double pDriveTrainPowerLow) {
            logLevel = RobotLogCommon.CommonLogLevel.valueOf(pLogLevel);
            driveTrainPowerHigh = pDriveTrainPowerHigh;
            driveTrainPowerLow = pDriveTrainPowerLow;
        }
    }

}

