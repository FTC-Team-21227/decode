package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * An extended Servo wrapper class which implements utility features such as
 * caching to reduce loop times and custom position clipping.
 */
public class CachedServo {
    private double lastPos = Double.NaN;
    private final Servo servo;
    private double posThreshold = 0.0001;

    public CachedServo(Servo servo, double posThreshold) {
        this.servo = servo;
        this.posThreshold = posThreshold;
    }

    public CachedServo(Servo servo) {
        this.servo = servo;
    }

    public Servo getServo(){
        return servo;
    }
    public void scaleRange(double min, double max) {
        servo.scaleRange(min, max);
    }
    public void setPosition(double pos) {
        if (Double.isNaN(lastPos) || (Math.abs(this.lastPos - pos) > this.posThreshold)) {
            lastPos = pos;
            servo.setPosition(constrain(pos));
        }
    }

    public double getPosition() {
        return(servo.getPosition());
    }

    public void setDirection(Servo.Direction direction) {
        this.servo.setDirection(direction);
    }

    public void setCachingThreshold(double posThreshold) {
        this.posThreshold = posThreshold;
    }

    public double getPos() {
        return lastPos;
    }


    public double constrain(double pos){
        return Range.clip(pos,0,1);
    }

    // Returns true if turret target position is out of range
    public boolean commandedOutsideRange(){
        return (lastPos>=0.9999999 || lastPos<=0.0000001); //changed to >=, <=
    }
}
