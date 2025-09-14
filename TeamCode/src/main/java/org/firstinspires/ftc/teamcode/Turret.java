package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret {
    Servo left;
    Servo right;
    public Turret(HardwareMap hardwareMap){
        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");
        left.scaleRange(0,1); // left = 0, right = 2pi
        right.scaleRange(0,1);
    }
    public void turnToRobotAngle(double angle){ //radians (normalized)
        double targetPos = angle/2/Math.PI;
        left.setPosition(targetPos);
        right.setPosition(targetPos);
    }
    public double getTurretRobotAngle(){
        return left.getPosition()*2*Math.PI;
    }
}
