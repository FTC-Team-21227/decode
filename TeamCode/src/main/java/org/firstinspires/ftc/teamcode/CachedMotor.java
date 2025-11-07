package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * An extended motor class that utilizes more features than the
 * regular motor.
 */
public class CachedMotor {
    private double lastPower = Double.NaN;
    private final DcMotorEx motor;
    private double powerThreshold = 0.001;

    public CachedMotor(DcMotorEx motor, double powerThreshold) {
        this.motor = motor;
        this.powerThreshold = powerThreshold;
    }

    public CachedMotor(DcMotorEx motor) {
        this.motor = motor;
    }

    public DcMotorEx getMotor(){
        return motor;
    }

    public void setPower(double power) {
        if (Double.isNaN(lastPower) || (Math.abs(this.lastPower - power) > this.powerThreshold) || (power == 0 && lastPower != 0)) {
            lastPower = power;
            motor.setPower(power);
        }
//        else { //setVelocity() doesn't get cached
//            RobotLog.a("Cached!");
//        }
    }

    public int getPosition() {
        return(motor.getCurrentPosition());
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        this.motor.setDirection(direction);
    }

    public void setCachingThreshold(double powerThreshold) {
        this.powerThreshold = powerThreshold;
    }

    public double getPower() {
        return lastPower;
    }

    public void setMode(DcMotorEx.RunMode runMode) {
        this.motor.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior) {
        this.motor.setZeroPowerBehavior(zeroPowerBehavior);
    }
}
