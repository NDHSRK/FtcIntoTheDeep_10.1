package org.firstinspires.ftc.teamcode.robot.device.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;

import java.util.EnumMap;
import java.util.Objects;
import java.util.Optional;

import javax.xml.xpath.XPathExpressionException;

public abstract class MultiMotorCore {

    private static final String TAG = MultiMotorCore.class.getSimpleName();

    protected EnumMap<FTCRobot.MotorId, DcMotorEx> motorMap = new EnumMap<>(FTCRobot.MotorId.class);

    protected final double clicksPerMotorRev;
    protected final double maxVelocity; // clicks per second

    public MultiMotorCore(HardwareMap pHardwareMap, XPathAccess pConfigXPath, String pMotorElementName, FTCRobot.MotorId... pMotorIds) throws XPathExpressionException {
        clicksPerMotorRev = pConfigXPath.getRequiredDouble("multiple_motors" + "/@clicks_per_motor_rev");
        double motorRPM = pConfigXPath.getRequiredDouble("multiple_motors" + "/@rpm");
        maxVelocity = Math.floor((clicksPerMotorRev * motorRPM) / 60); // clicks per second

        // Get the configuration from RobotConfig.xml.
        RobotLogCommon.c(TAG, "Defining the motors for the " + pMotorElementName);
        RobotLogCommon.c(TAG, "Model " + pConfigXPath.getRequiredText("multiple_motors/@model"));

        for (FTCRobot.MotorId motorId: pMotorIds) {
            // The motor element name is the lowercase version of the MotorId.
            String oneMotorPath = "multiple_motors/" + motorId.toString().toLowerCase();
            RobotLogCommon.c(TAG, "Configuring motor " + oneMotorPath);

            DcMotorEx oneMotor = pHardwareMap.get(DcMotorEx.class, pConfigXPath.getRequiredText(oneMotorPath  + "/@device_name"));
       
            // Set the direction of the motor.
           oneMotor.setDirection(DcMotor.Direction.valueOf(pConfigXPath.getRequiredText(oneMotorPath + "/@direction")));

           motorMap.put(motorId, oneMotor);
        }
    }

    public double getClicksPerMotorRev() {
        return clicksPerMotorRev;
    }

    public DcMotor.RunMode getRunMode(FTCRobot.MotorId pMotorId) {
        return Objects.requireNonNull(motorMap.get(pMotorId),
                TAG + " getRunMode: motor " + pMotorId + " is not in the current configuration").getMode();
    }

    public void setRunModeAll(DcMotor.RunMode pMode) {
        motorMap.forEach((k, v) -> v.setMode(pMode));
    }

    public void setZeroPowerBrakeAll() {
        motorMap.forEach((k, v) -> v.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));
    }

    public void setZeroPowerBrakeBehaviorAll(DcMotor.ZeroPowerBehavior pBehavior) {
        motorMap.forEach((k, v) -> v.setZeroPowerBehavior(pBehavior));
    }

    public EnumMap<FTCRobot.MotorId, Integer> getCurrentPositions() {
        EnumMap<FTCRobot.MotorId, Integer> currentPositions = new EnumMap<>(FTCRobot.MotorId.class);

        motorMap.forEach((k, v) -> currentPositions.put(k, v.getCurrentPosition()));
        return currentPositions;
    }

    public EnumMap<FTCRobot.MotorId, Integer> getTargetPositions() {
        EnumMap<FTCRobot.MotorId, Integer> targetPositions = new EnumMap<>(FTCRobot.MotorId.class);

        motorMap.forEach((k, v) -> targetPositions.put(k, v.getTargetPosition()));
        return targetPositions;
    }

    public void setTargetPositions(int pPosition) {
        motorMap.forEach((k, v) -> v.setTargetPosition(pPosition));
    }

    // Run one of the multiple motors at the requested velocity.
    public void runAtVelocity(FTCRobot.MotorId pMotorId, double pVelocity) {
        DcMotorEx motor = Objects.requireNonNull(motorMap.get(pMotorId),
                TAG + " runAtVelocity: motor " + pMotorId + " is null in the motorMap");

        RobotLogCommon.d(TAG,"Motor " + pMotorId + " run mode " + motor.getMode()); // just log the run mode
        RobotLogCommon.d(TAG, "Running at ticks per second " + (pVelocity * maxVelocity));
        Objects.requireNonNull(motorMap.get(pMotorId),
                TAG + " setTargetPosition: motor " + pMotorId + " is null in pVelocityMap").setVelocity(pVelocity * maxVelocity);
    }

    public void runAtVelocityAll(double pVelocity) {
        motorMap.forEach((k, v) ->
        {
            RobotLogCommon.d(TAG,"Motor " + k + " run mode " + v.getMode()); // just log the run mode
            v.setVelocity(pVelocity * maxVelocity);
        });
    }

    public boolean allBusy() {
        Optional<Boolean> atLeastOneMotorNotBusy = Optional.of(motorMap.entrySet().stream()
                .anyMatch(entry -> !entry.getValue().isBusy())); // at least one motor is not busy

        return !atLeastOneMotorNotBusy.get();
    }

    // For power, one motor might run at positive power, the other negative.
    public void runAtPowerAll(EnumMap<FTCRobot.MotorId, Double> pPowerMap) {
        pPowerMap.forEach((k, v) ->
        {
            DcMotorEx motor = Objects.requireNonNull(motorMap.get(k),
                    TAG + " runAtVelocity: motor " + k + " is null in the motorMap");

            RobotLogCommon.d(TAG,"Motor " + k + " run mode " + motor.getMode()); // just log the run mode
            motor.setPower(v);
        });
    }

    public void stopAllZeroVelocity() {
        RobotLogCommon.d(TAG, "Stop all motors - set velocity to zero");
        motorMap.forEach((k, v) -> v.setVelocity(0.0));
    }

    public void stopAllZeroPower() {
        RobotLogCommon.d(TAG, "Stop all motors - set power to zero");
        motorMap.forEach((k, v) -> v.setVelocity(0.0));
    }

}
