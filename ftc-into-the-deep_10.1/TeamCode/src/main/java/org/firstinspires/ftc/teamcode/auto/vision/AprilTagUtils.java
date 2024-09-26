package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;

import java.util.EnumSet;
import java.util.Optional;

public class AprilTagUtils {

    private static final String TAG = AprilTagUtils.class.getSimpleName();

    //## Even though the AprilTag ids below are specific to the CenterStage
    // game, retain this enum in the Core project as a model.

    // AprilTag identifiers
    public enum AprilTagId {
        TAG_ID_11(11), TAG_ID_12(12), TAG_ID_13(13),
        TAG_ID_14(14), TAG_ID_15(15), TAG_ID_16(16);

        private final int numericAprilTagId;

        AprilTagId(int pNumericId) {
            numericAprilTagId = pNumericId;
        }

        public int getNumericId() {
            return numericAprilTagId;
        }

        // Given the numeric id of an AprilTag return its enumeration.
        // See https://stackoverflow.com/questions/27807232/finding-enum-value-with-java-8-stream-api
        public static AprilTagId getEnumValue(int pNumericId) {
            Optional<AprilTagId> matchingTag = EnumSet.allOf(AprilTagId.class).stream()
                    .filter(tag -> tag.getNumericId() == pNumericId)
                    .findFirst();

            return matchingTag.orElseThrow(() -> new AutonomousRobotException(TAG, "Invalid AprilTag number " + pNumericId));
        }
    }

}
