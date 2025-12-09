package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.tuning.TestBenchColor;

public class ColorSensorTest extends OpMode {
    TestBenchColor bench = new TestBenchColor();
    TestBenchColor.DetectedColor detectedColor;

    @Override
    public void init()
    {
        bench.init(hardwareMap);
    }

    @Override
    public void loop()
    {
        detectedColor = bench.getDetectedColor(telemetry);
        telemetry.addData("color detected: ", detectedColor);
    }
}