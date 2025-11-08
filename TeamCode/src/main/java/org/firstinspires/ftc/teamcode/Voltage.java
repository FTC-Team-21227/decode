package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * An extended Voltage wrapper class that prevents multiple reads in the same loop.
 */
public class Voltage {
    private double lastReading = 12;
    private final VoltageSensor v;

    public Voltage(VoltageSensor v) {
        this.v = v;
    }

    public VoltageSensor getSensor(){
        return v;
    }

    public double updateVoltage() {
        lastReading = v.getVoltage();
        return lastReading;
    }
    public double getVoltage() {
        return lastReading;
    }
}
