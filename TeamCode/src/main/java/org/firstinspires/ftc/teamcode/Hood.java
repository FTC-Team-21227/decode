package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hood {
    Servo HOOD;
    public Hood(HardwareMap hardwareMap){
        HOOD = hardwareMap.get(Servo.class, "hood");
        HOOD.scaleRange(Robot.Constants.hoodScale0,Robot.Constants.hoodScale1); // left = low, right = high
    }
    public void turnToAngle(double angle){ //range 0-some angle
        HOOD.setPosition(angle/(Robot.Constants.hoodHighAngle-Robot.Constants.hoodLowAngle));
    }
    public double getAngle(){
        return HOOD.getPosition()*(Robot.Constants.hoodHighAngle-Robot.Constants.hoodLowAngle);
    }
}
