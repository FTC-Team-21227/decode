package org.firstinspires.ftc.teamcode.tuning;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestBenchColor {
    NormalizedColorSensor colorSensor;

    public enum DetectedColor
    {
//        RED,
//        BLUE,
//        YELLOW,
        GREEN,
        PURPLE,
        UNKNOWN
    }

    public void init(HardwareMap hwMap)
    {
        colorSensor = hwMap.get(NormalizedColorSensor.class, "colorSensor");
        colorSensor.setGain(1); // TEST SATURATION
    }

    public DetectedColor getDetectedColor(Telemetry telemetry)
    {
        NormalizedRGBA colors = colorSensor.getNormalizedColors(); // returns 4 values

        float normRed, normGreen, normBlue; // normalize colors
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;

        telemetry.addData("red", normRed);
        telemetry.addData("green", normGreen);
        telemetry.addData("blue", normBlue);

        // TODO add if statements for specific colors
        /* range following colors
        x red, green, blue
        RED =
        GREEN =
        BLUE =
         */

        if (normRed< 0 && normGreen < 0 && normBlue > 1) // TUNE VALUES
        {
            return DetectedColor.GREEN;
        }
        else if (normRed > 1 && normGreen < 0 && normBlue > 1) // TUNE VALUES
        {
            return DetectedColor.PURPLE;
        }

        return DetectedColor.UNKNOWN;
    }
}
