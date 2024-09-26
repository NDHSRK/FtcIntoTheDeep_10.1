package org.firstinspires.ftc.teamcode.robot.device.motor;

import static android.os.SystemClock.sleep;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.auto.FTCAuto;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;

import java.util.EnumMap;

// Move two motors in tandem.
public class ElevatorMotion {

    private static final String TAG = ElevatorMotion.class.getSimpleName();

    public enum ElevatorAction {MOVE_AND_HOLD_VELOCITY, MOVE_AND_STOP}

    private final LinearOpMode linearOpMode;
    private final Elevator elevator;
    private boolean abnormalTermination = false;

    public ElevatorMotion(LinearOpMode pLinearOpMode, Elevator pElevator) {
        linearOpMode = pLinearOpMode;
        elevator = pElevator;

        // Set the run mode for both motors.
        //## Follow the FTC sample PushbotAutoDriveByEncoder_Linear and always
        // set the run modes in this order: STOP_AND_RESET_ENCODER,
        // RUN_USING_ENCODER. Then call setTargetPosition followed by a run mode
        // of RUN_TO_POSITION. In this class we only set STOP_AND_RESET_ENCODER
        // once because we'll always run to an absolute position.
        elevator.setRunModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setRunModeAll(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetAndMoveElevator(int pTargetPosition, double pVelocity, ElevatorAction pMotorAction) {
        elevator.setRunModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        moveElevator(pTargetPosition, pVelocity, pMotorAction);
    }

    // Note: all target positions are absolute.
    @SuppressLint("DefaultLocale")
    public void moveElevator(int pTargetPosition, double pVelocity, ElevatorAction pElevatorAction) {
        abnormalTermination = false;

        EnumMap<FTCRobot.MotorId, Integer> currentMotorPositions = elevator.getCurrentPositions();

        currentMotorPositions.forEach((k, v) ->
        RobotLogCommon.d(TAG, k + " current position " + v + ", target position " + pTargetPosition));

        double velocity = Math.abs(pVelocity); // velocity is always positive; position determines direction
        RobotLogCommon.d(TAG, "Move elevator to position " + pTargetPosition + ", at velocity " + velocity);

        elevator.setTargetPositions(pTargetPosition);
        elevator.setRunModeAll(DcMotor.RunMode.RUN_TO_POSITION);

        // Start moving.
        elevator.runAtVelocityAll(velocity);

        // Keep moving until one of the motors has reached its target position.
        try {
            while (elevator.allBusy()) {
                if (!linearOpMode.opModeIsActive()) {
                    RobotLog.ee(TAG, "OpMode went inactive during movement of the elevator");
                    RobotLogCommon.d(TAG, "OpMode went inactive during movement of the elevator");
                    break;
                }

                // If we're running Autonomous check the timer.
                if (FTCAuto.autonomousTimer != null && FTCAuto.autonomousTimer.autoTimerIsExpired()) {
                    RobotLog.dd(TAG, "Autonomous panic stop triggered during movement of the elevator");
                    // Do not set the abnormalTermination flag - we want the code in finally() to run.
                    break;
                }
            }
        } catch (Exception ex) {
            abnormalTermination = true;
            throw ex;
        } finally {
            if (!linearOpMode.opModeIsActive() || abnormalTermination) {
                RobotLog.ee(TAG, "Abnormal termination");
            } else {

                // Only stop the motors if the user has requested a stop; otherwise hold their position.
                if (pElevatorAction == ElevatorAction.MOVE_AND_STOP)
                    elevator.stopAllZeroVelocity();

                // Log ending click counts for both motors.
                RobotLogCommon.d(TAG, "Elevator motion complete");
                EnumMap<FTCRobot.MotorId, Integer> elevatorMotorPositions = elevator.getCurrentPositions();
                RobotLogCommon.d(TAG, "Elevator motor " + FTCRobot.MotorId.LEFT_ELEVATOR +
                        " ending position " + elevatorMotorPositions.get(FTCRobot.MotorId.LEFT_ELEVATOR));
                RobotLogCommon.d(TAG, "Elevate motor " + FTCRobot.MotorId.RIGHT_ELEVATOR +
                        " ending position " + elevatorMotorPositions.get(FTCRobot.MotorId.RIGHT_ELEVATOR));
            }
        }
    }

    // Move both motors downward until they each trip their respective magnetic switch.
    public void moveElevatorMotorsDownToMagneticLimit(TouchSensor pLeftSwitch, TouchSensor pRightSwitch, double pVelocity) {
        elevator.setRunModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotLogCommon.d(TAG, "Moving elevator motors " + FTCRobot.MotorId.LEFT_ELEVATOR + ", " +
                FTCRobot.MotorId.RIGHT_ELEVATOR + " down to their magnetic limit");

        try {
            boolean reachedLeftLimit = false;
            boolean reachedRightLimit = false;
            double velocity = -Math.abs(pVelocity); // velocity is always negative
            elevator.runAtVelocityAll(velocity); // start moving

            while (!reachedLeftLimit || !reachedRightLimit) {
                if (!reachedLeftLimit && pLeftSwitch.isPressed()) {
                    reachedLeftLimit = true;
                    elevator.runAtVelocity(FTCRobot.MotorId.LEFT_ELEVATOR, 0.0);
                }

                if (!reachedRightLimit && pRightSwitch.isPressed()) {
                    reachedRightLimit = true;
                    elevator.runAtVelocity(FTCRobot.MotorId.RIGHT_ELEVATOR, 0.0);
                }

                sleep(10);
            }
        } finally {
            elevator.stopAllZeroVelocity(); // for safety in case of an exception

            RobotLogCommon.d(TAG, "Motion of elevators down to their magnetic limits complete");
            EnumMap<FTCRobot.MotorId, Integer> elevatorMotorPositions = elevator.getCurrentPositions();
            RobotLogCommon.d(TAG, "Elevator motor " + FTCRobot.MotorId.LEFT_ELEVATOR +
                    " ending position " + elevatorMotorPositions.get(FTCRobot.MotorId.LEFT_ELEVATOR));
            RobotLogCommon.d(TAG, "Elevate motor " + FTCRobot.MotorId.RIGHT_ELEVATOR +
                    " ending position " + elevatorMotorPositions.get(FTCRobot.MotorId.RIGHT_ELEVATOR));

            // Reset the zero point of the encoders.
            elevator.setRunModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

}

