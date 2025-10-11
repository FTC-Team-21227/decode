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
    public void turnToAngle(double angle){ // range 50 - 87 degrees
        HOOD.setPosition(constrain((angle - Robot.Constants.hoodLowAngle) / (Robot.Constants.hoodHighAngle - Robot.Constants.hoodLowAngle)));
    }
    //
    public double getAngle(){
        return HOOD.getPosition() * (Robot.Constants.hoodHighAngle - Robot.Constants.hoodLowAngle) + Robot.Constants.hoodLowAngle;
    }
    public double constrain(double pos){
        if (pos > 1){
            pos = 0.9999999;
        }
        else if (pos < 0){
            pos = 0.0000001;
        }
        return pos;
    }
    public boolean commandedOutsideRange(){
        return (HOOD.getPosition()==0.9999999 || HOOD.getPosition()==0.0000001);
    }
}
