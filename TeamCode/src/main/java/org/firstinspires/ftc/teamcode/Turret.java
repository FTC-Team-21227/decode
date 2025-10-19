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
    }

    /**
     * Turns turret to the robot-relative angle
     * @param angle in radians
     */
    public void turnToRobotAngle(double angle) {
//        double targetPos = angle / (2 * Math.PI);
        angle = (AngleUnit.normalizeRadians(angle) - Robot.Constants.turretTargetRangeOffset + Math.PI) % (2 * Math.PI) + Robot.Constants.turretTargetRangeOffset - Math.PI;//+Math.PI)%(2*Math.PI)-Math.PI;
        turret.setPosition(constrain((angle - Robot.Constants.turretLowAngle) / (Robot.Constants.turretHighAngle - Robot.Constants.turretLowAngle)));
    }


    /**
     * @return Turret's robot-relative angle (in radians)
     */
    public double getTurretRobotAngle() {
        return turret.getPosition() * (Robot.Constants.turretHighAngle - Robot.Constants.turretLowAngle) + Robot.Constants.turretLowAngle;
    }

    /**
     * @return Turret's robot-relative pose based on its position and current heading
     */
    public Pose2d getPoseRobotTurret() {
        return new Pose2d(Robot.Constants.turretPos,getTurretRobotAngle());
    }

    /**
     * Turret will not move if set past its limit, so this function returns the closest position that is still in range
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
     * @return true if turret target position is out of range
     */
    public boolean commandedOutsideRange(){
        return (turret.getPosition()==0.9999999 || turret.getPosition()==0.0000001);
    }
}
