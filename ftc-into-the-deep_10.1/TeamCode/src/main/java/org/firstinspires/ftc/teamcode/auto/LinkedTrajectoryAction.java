package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

//**TODO This class will not be needed if the fresh() ActionBuilder call
// works correctly - see the updated CenterStage example.

// This class is necessary when we need to get the ending pose from the
// previous trajectory after that trajectory has completed. That pose,
// which is stored in the MecanumDrive class, is only available at the
// time the run method below is invoked. We discovered this problem
// during testing with the following sequence of actions.
/*
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(14.76, -62.89, Math.toRadians(90)));

        // This is the first trajectory Action after the creation of the instance
        // of MecanumDrive so the use of drive.pose is ok here.
        Action toSubmersible = TrajectoryActionCollection.buildTrajectoryAction(drive, drive.pose, TrajectoryActionCollection.TrajectoryActionId.RED_F4_TO_SUBMERSIBLE);

        // The field drive.pose is not actually used here. This Action is just a
        // placeholder.
        Action hangSpecimen = drive.actionBuilder(drive.pose)
                .waitSeconds(2)
                .build();

        // This Action depends on the ending pose of toSubmersible, not drive.pose as it
        // is captured here at the time toSample1 is constructed.
        Action toSample1 = drive.actionBuilder(drive.pose)
                .lineToY(-48)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(37.11, -37.11, Math.toRadians(45)), Math.toRadians(45))
                .waitSeconds(2)
                .build();
 */

//**TODO Archive this class if the call to .fresh() works.
// Use this class to defer the construction of an Action until the end pose of
// the previous Action has been set. Typically used in a SequentialAction.
public class LinkedTrajectoryAction implements Action {

    private final MecanumDrive drive;
    private final TrajectoryActionCollection.TrajectoryActionId trajectoryActionId;
    private Action action; // this is the nested action

    public LinkedTrajectoryAction(MecanumDrive pDrive, TrajectoryActionCollection.TrajectoryActionId pTrajectoryActionId) {
        drive = pDrive;
        trajectoryActionId = pTrajectoryActionId;
    }

    private boolean initialized = false;

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (!initialized) {
            initialized = true;

            //## A more generic - but maybe too generic - way of capturing a
            // reference to the actionBuilder method in MecanumDrive.
            // Function<Pose2d, TrajectoryActionBuilder> tab = drive::actionBuilder;

            RobotLog.dd("NestedAction", "Starting pose " + drive.pose);

            // We need a reference to MecanumDrive to access to the method
            // actionBuilder. We also extract the pose from the same instance -
            // needed for compatibility with other callers that may supply a
            // hard-coded pose to buildTrajectoryAction.
            action = TrajectoryActionCollection.buildTrajectoryAction(drive, drive.pose, trajectoryActionId);
        }

        if (!action.run(packet)) {
            RobotLog.dd("NestedAction", "Ending pose " + drive.pose);
            return false; // keep going
        }

        return true; // done
    }

}
