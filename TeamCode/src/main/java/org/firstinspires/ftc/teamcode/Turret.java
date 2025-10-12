package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret {
    Servo turret;

    public Turret(HardwareMap hardwareMap){
        turret = hardwareMap.get(Servo.class, "Shooter_Angle"); // Turntable
        turret.scaleRange(Robot.Constants.turretScale0,Robot.Constants.turretScale1); // 0 = +90 deg, 1 = -330 deg
    }

    /**
     * Turns turret to the robot-relative angle
     * @param angle in radians
     */
    public void turnToRobotAngle(double angle) {
//        double targetPos = angle / (2 * Math.PI);
        turret.setPosition((angle-Robot.Constants.turretLowAngle)/ (Robot.Constants.turretHighAngle - Robot.Constants.turretLowAngle));
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
}
