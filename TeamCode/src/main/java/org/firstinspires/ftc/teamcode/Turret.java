package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret {
    Servo left;
    Servo right;
    public Turret(HardwareMap hardwareMap){
        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");
        left.scaleRange(Robot.Constants.turretLeftScale0,Robot.Constants.turretLeftScale1); // left = 0, right = 2pi
        right.scaleRange(Robot.Constants.turretRightScale0,Robot.Constants.turretRightScale1);
    }
    public void turnToRobotAngle(double angle){ //radians (normalized)
        double targetPos = angle/2/Math.PI;
        left.setPosition(targetPos);
        right.setPosition(targetPos);
    }
    public double getTurretRobotAngle(){
        return left.getPosition()*2*Math.PI;
    }
    public Pose2d getPoseRobotTurret(){
        return new Pose2d(Robot.Constants.turretPos,getTurretRobotAngle());
    }
}
