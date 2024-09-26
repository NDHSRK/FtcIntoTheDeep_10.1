package org.firstinspires.ftc.teamcode.robot.device.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;

import javax.xml.xpath.XPathExpressionException;

public abstract class SingleMotorCore {

    private static final String TAG = SingleMotorCore.class.getSimpleName();

    protected final FTCRobot.MotorId motorId;
    protected final DcMotorEx singleMotor;

    protected final double clicksPerMotorRev;
    protected final double maxVelocity; // clicks per second

    public SingleMotorCore(HardwareMap pHardwareMap, XPathAccess pConfigXPath, String pMotorElementName, FTCRobot.MotorId pMotorId) throws XPathExpressionException {
        clicksPerMotorRev = pConfigXPath.getRequiredDouble("single_motor" + "/@clicks_per_motor_rev");
        double motorRPM = pConfigXPath.getRequiredDouble("single_motor" + "/@rpm");
        maxVelocity = Math.floor((clicksPerMotorRev * motorRPM) / 60); // clicks per second

        // Get the configuration from RobotConfig.xml.
        RobotLogCommon.c(TAG, "Defining the motor for the " + pMotorElementName);
        RobotLogCommon.c(TAG, "Model " + pConfigXPath.getRequiredText("single_motor/@model"));

        motorId = pMotorId;
        String singleMotorPath = "single_motor/" + motorId.toString().toLowerCase();
        singleMotor = pHardwareMap.get(DcMotorEx.class, pConfigXPath.getRequiredText(singleMotorPath  + "/@device_name"));
       
        // Set the direction of the motor.
        singleMotor.setDirection(DcMotor.Direction.valueOf(pConfigXPath.getRequiredText(singleMotorPath + "/@direction")));
    }

    public FTCRobot.MotorId getMotorId() {
        return motorId;
    }

    public DcMotor.RunMode getRunMode() {
        return singleMotor.getMode();
    }

    public void setRunMode(DcMotor.RunMode pMode) {
        singleMotor.setMode(pMode);
    }

    public void setZeroPowerBrake() {
        singleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setZeroPowerBrakeBehavior(DcMotor.ZeroPowerBehavior pBehavior) {
        singleMotor.setZeroPowerBehavior(pBehavior);
    }

    public int getCurrentPosition() {
        return singleMotor.getCurrentPosition();
    }

    public int getTargetPosition() {
        return singleMotor.getTargetPosition();
    }

    public void setTargetPosition(int pTargetClicks) {
        singleMotor.setTargetPosition(pTargetClicks);
    }

    // Assumes all clipping and all final modifications to the velocity,
    // e.g. running at .5 velocity, have already been performed.
    public void runAtVelocity(double pVelocity) {
        RobotLogCommon.d(TAG,"Motor " + motorId + " run mode " + singleMotor.getMode()); // just log the run mode
        RobotLogCommon.d(TAG, "Running at ticks per second " + (pVelocity * maxVelocity));
        singleMotor.setVelocity(pVelocity * maxVelocity);
    }

    public boolean isBusy() {
        if (singleMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
            throw new AutonomousRobotException(TAG, "Illegal test of isBusy((); motor " + motorId +
                    " has not been set to RUN_TO_POSITION; motor mode is " + singleMotor.getMode());

        return singleMotor.isBusy();
    }

    // For use with DcMotor.RunMode.RUN_WITHOUT_ENCODER.
    // Assumes all clipping and all final modifications to the power,
    // e.g. running at .5 power, have already been performed.
    //## Note that with either of the run modes RUN_USING_ENCODER or
    // RUN_TO_POSITION, setPower has no effect!!
    public void runAtPower(double pPower) {
        if (singleMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            throw new AutonomousRobotException(TAG, "Motor " + motorId + ": setPower requires RUN_WITHOUT_ENCODER");

        singleMotor.setPower(pPower);
    }

}
