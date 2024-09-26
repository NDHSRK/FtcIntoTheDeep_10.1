package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class TrajectoryActionCollection {

    private static final String TAG = TrajectoryActionCollection.class.getSimpleName();

    public enum TrajectoryActionId {RED_F4_TO_SUBMERSIBLE, RED_F4_TO_SAMPLE_1 }

    //## An example of how to use a Functional interface to get a reference to
    // the actionBuilder method in MecanumDrive.
    /*
    public static Action buildTrajectoryAction(Function<Pose2d, TrajectoryActionBuilder> pTrajectoryActionBuilder,
                                 Pose2d pPose,
                                 TrajectoryActionId pTrajectoryActionId) {
        // hard-coded example.
        return pTrajectoryActionBuilder.apply(pPose)
                .splineToSplineHeading(new Pose2d(6.44, -35.0, Math.toRadians(90)), Math.toRadians(90))
                .build();
    }
     */

    // Even though the pose is a public field in MecanumDrive the caller
    // the caller may choose to use a hard-coded pose.
    public static Action buildTrajectoryAction(MecanumDrive pDrive, Pose2d pPose,
                                               TrajectoryActionId pTrajectoryActionId) {
        switch (pTrajectoryActionId) {
            case RED_F4_TO_SUBMERSIBLE: {
                return pDrive.actionBuilder(pPose)
                        .splineToSplineHeading(new Pose2d(6.44, -35.0, Math.toRadians(90)), Math.toRadians(90))
                        .build();
            }
            case RED_F4_TO_SAMPLE_1: {
                return pDrive.actionBuilder(pPose)
                        .lineToY(-48)
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(37.11, -37.11, Math.toRadians(45)), Math.toRadians(45))
                        .waitSeconds(2)
                        .build();
            }
            default: {
                throw new AutonomousRobotException(TAG, "Unrecognized trajectory action id " + pTrajectoryActionId);
            }
        }
    }
}
