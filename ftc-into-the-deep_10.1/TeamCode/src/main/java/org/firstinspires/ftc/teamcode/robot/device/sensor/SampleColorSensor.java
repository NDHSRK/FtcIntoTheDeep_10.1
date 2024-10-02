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

    public Pair<SampleColor, Double> getColorAndDistance() {
        SampleColor retColor = SampleColor.NPOS;
        double retDistance =  DISTANCE_NPOS;

        // Get the distance from the sensor.
        double distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
        if (distance >= distance_low && distance <= distance_high)
            retDistance = distance;

        // Get the normalized colors from the sensor.
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        // Update the hsvValues array by passing it to Color.colorToHSV().
        float[] hsv = new float[3];
        Color.colorToHSV(colors.toColor(), hsv);
        float hueOfSample = hsv[0];

        // Match the returned hue against our ranges.
        if (getColorOfSample(hueOfSample, blue_low, blue_high))
            retColor = SampleColor.BLUE;
        else if (getColorOfSample(hueOfSample, red_low, red_high))
            retColor = SampleColor.RED;
        else if (getColorOfSample(hueOfSample, yellow_low, yellow_high))
            retColor = SampleColor.YELLOW;

        RobotLogCommon.d(TAG, "Color sensor hue: " + retColor + ", distance " +  retDistance);

        return Pair.create(retColor, retDistance);
    }

    private boolean getColorOfSample(float pHueOfSample, double pLow, double pHigh) {
        return ((pHueOfSample >= pLow && pHueOfSample <= pHigh) || // normal case
                (pLow > pHigh) && ((pHueOfSample >= pLow && pHueOfSample <= 360) || // range spans 0
                        (pHueOfSample >= 0 && pHueOfSample <= pHigh)));
    }

}

