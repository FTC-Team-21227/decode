package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hood {
    Servo HOOD;

    public Hood(HardwareMap hardwareMap){
        HOOD = hardwareMap.get(Servo.class, "HoodLinearServo");
        HOOD.setDirection(Servo.Direction.FORWARD);
        HOOD.scaleRange(Robot.Constants.hoodScale0, Robot.Constants.hoodScale1); // left = low, right = high
    }

    /**
     * Sets hood position to a certain angle (relative to ground/horizontal)
     * @param angle angle in radians
     */
    public void turnToAngle(double angle){ // Range: 50-87 degrees (ball trajectory, relative to ground)
        HOOD.setPosition(constrain((angle - Robot.Constants.hoodLowAngle) / (Robot.Constants.hoodHighAngle - Robot.Constants.hoodLowAngle)));
    }

    /**
     * Returns angle of hood (radians)
     */
    public double getAngle(){
        return HOOD.getPosition() * (Robot.Constants.hoodHighAngle - Robot.Constants.hoodLowAngle) + Robot.Constants.hoodLowAngle;
    }

    /**
     * Hood will not move if set past its limit, so this function returns the closest position that is still in range
     * @param pos original position
     */
    public double constrain(double pos){
        if (pos > 1){
            pos = 0.9999999;
        }
        else if (pos < 0){
            pos = 0.0000001;
        }
        return pos;
    }

    /**
     * @return true if hood target position is out of range
     */
    public boolean commandedOutsideRange(){
        return (HOOD.getPosition()==0.9999999 || HOOD.getPosition()==0.0000001);
    }
}
