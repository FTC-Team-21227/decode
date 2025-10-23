package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Turret {
    Servo turret;

    public Turret(HardwareMap hardwareMap){
        turret = hardwareMap.get(Servo.class, "turret"); // Turntable
        turret.scaleRange(Robot.Constants.turretScale0,Robot.Constants.turretScale1); // 0 = +90 deg, 1 = -330 deg
        // *** double check scaleRange
    }

    // Turns turret to the robot-relative angle in radians
    public void turnToRobotAngle(double angle) {
//        double targetPos = angle / (2 * Math.PI);
        angle = (AngleUnit.normalizeRadians(angle) - Robot.Constants.turretTargetRangeOffset + Math.PI)
                % (2 * Math.PI) + Robot.Constants.turretTargetRangeOffset - Math.PI;//+Math.PI)%(2*Math.PI)-Math.PI;
//        angle = AngleUnit.normalizeRadians(angle);
        turret.setPosition(constrain((angle - Robot.Constants.turretLowAngle) / (Robot.Constants.turretHighAngle - Robot.Constants.turretLowAngle)));
    }

    // Gives turret's robot-relative angle (in radians)
    public double getTurretRobotAngle() {
        return turret.getPosition() * (Robot.Constants.turretHighAngle - Robot.Constants.turretLowAngle) + Robot.Constants.turretLowAngle;
    }

    // Gives turret's robot-relative pose based on its position and current heading
    public Pose2d getPoseRobotTurret() {
        return new Pose2d(Robot.Constants.turretPos,getTurretRobotAngle());
    }

    // Function returns the closest position that is still in range (turret will not move if not in range)
    public double constrain(double ogPos){
        if (ogPos > 1){
            ogPos = 0.9999999;
        }
        else if (ogPos < 0){
            ogPos = 0.0000001;
        }
        return ogPos;
    }

    // Returns true if turret target position is out of range
    public boolean commandedOutsideRange(){
        return (turret.getPosition()>=0.9999999 || turret.getPosition()<=0.0000001); //changed to >=, <=
    }
}
