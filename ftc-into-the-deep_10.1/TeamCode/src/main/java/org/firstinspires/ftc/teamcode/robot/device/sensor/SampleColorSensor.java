package org.firstinspires.ftc.teamcode.robot.device.sensor;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;

import javax.xml.xpath.XPathExpressionException;

public class SampleColorSensor {
    private static final String TAG = SampleColorSensor.class.getSimpleName();

    public enum SampleColor {
        BLUE, RED, YELLOW, NPOS
    }

    public static final double DISTANCE_NPOS = -1.0;

    private final NormalizedColorSensor colorSensor;
    private final double blue_low;
    private final double blue_high;
    private final double red_low;
    private final double red_high;
    private final double yellow_low;
    private final double yellow_high;
    private final double distance_low;
    private final double distance_high;

    public SampleColorSensor(HardwareMap pHardwareMap, XPathAccess pConfigXPath, String pSensorElementName, FTCRobot.SensorId pSensorId) throws XPathExpressionException {

        // Get the configuration from RobotConfig.xml.
        RobotLogCommon.c(TAG, "Defining the sensor for the " + pSensorElementName);
        RobotLogCommon.c(TAG, "Model " + pConfigXPath.getRequiredText("sensor/@model"));

        colorSensor = pHardwareMap.get(NormalizedColorSensor.class, pConfigXPath.getRequiredText("sensor/" + pSensorId.toString().toLowerCase() + "/@device_name"));

        // Get the color and distance ranges.
        /*
        <hsv_hue_range>
         <blue>
           <low></low>
           <high></high>
         </blue>
         <red>
           <low></low>
           <high></high>
         </red>
         <yellow>
           <low></low>
           <high></high>
         </yellow>
        </hsv_hue_range>
        <distance_range>
           <low></low>
           <high></high>
        </distance_range>
       */

        // Extract values from RobotConfig.xml.
        blue_low = pConfigXPath.getRequiredDouble("hsv_hue_range/blue/low");
        blue_high = pConfigXPath.getRequiredDouble("hsv_hue_range/blue/high");
        red_low = pConfigXPath.getRequiredDouble("hsv_hue_range/red/low");
        red_high = pConfigXPath.getRequiredDouble("hsv_hue_range/red/high");
        yellow_low = pConfigXPath.getRequiredDouble("hsv_hue_range/yellow/low");
        yellow_high = pConfigXPath.getRequiredDouble("hsv_hue_range/yellow/high");

        distance_low = pConfigXPath.getRequiredDouble("distance_range/yellow/low");
        distance_high = pConfigXPath.getRequiredDouble("distance_range/yellow/high");
    }

    //**TODO Put the range-checking logic here.
    public Pair<SampleColor, Double> getColorAndDistance() {

        // Get the normalized colors from the sensor.
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        // Update the hsvValues array by passing it to Color.colorToHSV().
        float[] hsv = new float[3];
        Color.colorToHSV(colors.toColor(), hsv);

        //**TODO Match the returned hue against our ranges.
        // Watch out for a range that spans 0.
        //**TODO Log the final result, e.g.
        //   RobotLogCommon.d(TAG, "Color sensor hue in range for blue");

        // Get the distance from the sensor.
        double distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
        //**TODO return DISTANCE_NPOS if the distance is out of range.

        return Pair.create(SampleColor.NPOS, DISTANCE_NPOS); //**TODO TEMP
    }

}

