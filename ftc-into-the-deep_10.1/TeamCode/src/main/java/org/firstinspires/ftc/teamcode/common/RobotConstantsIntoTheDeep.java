package org.firstinspires.ftc.teamcode.common;

public class RobotConstantsIntoTheDeep {

    public enum OpMode {
        // Autonomous OpModes
        BLUE_A3(OpModeType.COMPETITION),
        BLUE_A4(OpModeType.COMPETITION),
        RED_F3(OpModeType.COMPETITION),
        RED_F4(OpModeType.COMPETITION),

        TEST(OpModeType.AUTO_TEST), TEST_PRE_MATCH(OpModeType.AUTO_TEST),
        TEST_ELEVATOR(OpModeType.AUTO_TEST),
        AUTO_NO_DRIVE(OpModeType.AUTO_TEST),

        // TeleOp OpModes
        TELEOP_NO_DRIVE(OpModeType.TELEOP_TEST),

        // Indication that an OpMode has not yet been assigned.
        OPMODE_NPOS(OpModeType.PSEUDO_OPMODE);

        public enum OpModeType {COMPETITION, AUTO_TEST, TELEOP_TEST, PSEUDO_OPMODE}
        private final OpModeType opModeType;

        OpMode(OpModeType pOpModeType) {
            opModeType = pOpModeType;
        }

        public OpModeType getOpModeType() {
            return opModeType;
        }
    }

    // The CameraId identifies each unique camera and its position on
    // the robot.
    public enum InternalWebcamId {
        FRONT_WEBCAM, REAR_WEBCAM, WEBCAM_NPOS
    }

    public enum ProcessorIdentifier {
        RAW_FRAME, APRIL_TAG, PROCESSOR_NPOS
    }

}