package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Turret {
    CachedServo turret;

    public Turret(HardwareMap hardwareMap){
//        turret = hardwareMap.get(Servo.class, "turret"); // Turntable
        turret = new CachedServo(hardwareMap.get(Servo.class,"turret"));
//        turret.scaleRange(Robot.Constants.turretScale0,Robot.Constants.turretScale1); // 0 = +90 deg, 1 = -330 deg
        turret.scaleRange(0,1); // 0 = +90 deg, 1 = -330 deg
        // *** double check scaleRange
    }

    // Turns turret to the robot-relative angle in radians
    public void turnToRobotAngle(double angle) {
//        double targetPos = angle / (2 * Math.PI);
//        angle = (AngleUnit.normalizeRadians(angle) - Robot.Constants.turretTargetRangeOffset + Math.PI)
//                % (2 * Math.PI) + Robot.Constants.turretTargetRangeOffset - Math.PI;//+Math.PI)%(2*Math.PI)-Math.PI;
//        angle = AngleUnit.normalizeRadians(angle);
        angle = AngleUnit.normalizeRadians(angle-Robot.Constants.turretTargetRangeOffset) + Robot.Constants.turretTargetRangeOffset;
        turret.setPosition(constrain(Range.scale(angle,Robot.Constants.turretLowAngle,Robot.Constants.turretHighAngle,Robot.Constants.turretScale0,Robot.Constants.turretScale1)));
//        turret.setPosition(/*constrain*/((angle - Robot.Constants.turretLowAngle) / (Robot.Constants.turretHighAngle - Robot.Constants.turretLowAngle)));
    }

    // Gives turret's robot-relative angle (in radians)
    public double getTurretRobotAngle() {
        return Range.scale(turret.getPosition(), Robot.Constants.turretScale0,Robot.Constants.turretScale1,Robot.Constants.turretLowAngle,Robot.Constants.turretHighAngle);
//        return turret.getPosition() * (Robot.Constants.turretHighAngle - Robot.Constants.turretLowAngle) + Robot.Constants.turretLowAngle;
    }

    // Gives turret's robot-relative pose based on its position and current heading
    public Pose2d getPoseRobotTurret() {
        return new Pose2d(Robot.Constants.turretPos.position,getTurretRobotAngle());
    }

    // Function returns the closest position that is still in range (turret will not move if not in range)
    public double constrain(double ogPos){
//        if (ogPos > 1){
//            ogPos = 0.9999999;
//        }
//        else if (ogPos < 0){
//            ogPos = 0.0000001;
//        }
//
        return Range.clip(ogPos,Robot.Constants.turretClip0,Robot.Constants.turretClip1);
    }

    // Returns true if turret target position is out of range
//    public boolean commandedOutsideRange(){
//        return (turret.getPosition()>=0.9999999 || turret.getPosition()<=0.0000001); //changed to >=, <=
//    }
}
