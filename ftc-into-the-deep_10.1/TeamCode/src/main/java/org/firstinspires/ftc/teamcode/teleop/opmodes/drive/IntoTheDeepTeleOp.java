package org.firstinspires.ftc.teamcode.teleop.opmodes.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.teleop.common.FTCButton;
import org.firstinspires.ftc.teamcode.teleop.common.FTCToggleButtonNWay;
import org.firstinspires.ftc.teamcode.teleop.common.TeleOpParallelDrive;
import org.firstinspires.ftc.teamcode.teleop.common.TeleOpWithAlliance;

import java.util.EnumSet;
import java.util.Objects;

public class IntoTheDeepTeleOp extends TeleOpWithAlliance {

    private static final String TAG = IntoTheDeepTeleOp.class.getSimpleName();

    // Drive train
    enum DrivePower {FULL_POWER, HALF_POWER}

    private final FTCToggleButtonNWay<DrivePower> toggleHalfPower;
    private double driveTrainPower;
    private double previousDriveTrainPower;
    private final double driveTrainPowerHigh;
    private final double driveTrainPowerLow;
    private final TeleOpParallelDrive parallelDrive;

    public IntoTheDeepTeleOp(RobotConstants.Alliance pAlliance,
                             LinearOpMode pLinearOpMode, FTCRobot pRobot) {
        super(pAlliance, pLinearOpMode, pRobot);
        RobotLogCommon.c(TAG, "Constructing IntoTheDeepTeleOp");
        RobotLogCommon.setMostDetailedLogLevel(Objects.requireNonNull(robot.teleOpSettings, TAG + " teleOpSettings unexpectedly null").logLevel);

        toggleHalfPower = new FTCToggleButtonNWay<>(pLinearOpMode, FTCButton.ButtonValue.GAMEPAD_1_A,
                EnumSet.allOf(DrivePower.class));

        driveTrainPowerHigh = robot.teleOpSettings.driveTrainPowerHigh;
        driveTrainPower = driveTrainPowerHigh;
        previousDriveTrainPower = driveTrainPower;
        driveTrainPowerLow = robot.teleOpSettings.driveTrainPowerLow;

        // Start the drive train in parallel.
        parallelDrive = new TeleOpParallelDrive(linearOpMode, robot.teleOpDriveTrain, driveTrainPower);
        RobotLogCommon.c(TAG, "Finished constructing CenterStageTeleOp");
    }

    @Override
    public void runTeleOp() throws Exception {
        try {
            // Safety check against the case where the driver hits the small stop
            // button during waitForStart(). We want to make sure that finally()
            // still runs. From the FTC SDK documentation for opModeIsActive():
            // "If this method returns false after waitForStart() has previously
            // been called, you should break out of any loops and allow the OpMode
            // to exit at its earliest convenience."
            if (!linearOpMode.opModeIsActive()) {
                //## Do *not* do this throw new AutonomousRobotException(TAG, "OpMode unexpectedly inactive in runTeleOp()");
                RobotLogCommon.e(TAG, "OpMode unexpectedly inactive in runTeleOp()");
                return;
            }

            //## The drive train thread must be started here because
            // only now does opModeIsActive() return true.
            parallelDrive.startDriveTrain();

            while (linearOpMode.opModeIsActive()) {
                updateButtons();
                updateActions();
            }
        } finally {
            RobotLogCommon.d(TAG, "In finally() block");
        }
    }

    // Update the state of the active buttons. This method should be
    // called once per cycle.
    private void updateButtons() {

        // Game Controller 1
        toggleHalfPower.update();
    }

    // Execute the actions controlled by Player 1 and Player 2.
    // This method should be called once per cycle.
    private void updateActions() {
        updateToggleSpeed();

        if (driveTrainPowerChanged()) {
            parallelDrive.setPower(driveTrainPower);
        }
    }

    private void updateToggleSpeed() {
        if (toggleHalfPower.is(FTCButton.State.TAP)) {
            DrivePower newDrivePower = toggleHalfPower.toggle();
            if (newDrivePower == DrivePower.FULL_POWER)
                driveTrainPower = driveTrainPowerHigh;
            else if (newDrivePower == DrivePower.HALF_POWER)
                driveTrainPower = driveTrainPowerLow;
        }
    }

    private boolean driveTrainPowerChanged() {
        if (driveTrainPower == previousDriveTrainPower)
            return false;

        previousDriveTrainPower = driveTrainPower;
        return true;
    }

}
