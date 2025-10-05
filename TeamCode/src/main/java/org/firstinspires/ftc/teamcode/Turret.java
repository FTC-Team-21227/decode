package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret {
    Servo turret;

    public Turret(HardwareMap hardwareMap){
        turret = hardwareMap.get(Servo.class, "Shooter_Angle"); // Turntable
        turret.scaleRange(Robot.Constants.turretScale0,Robot.Constants.turretScale1); // Left = 0, right = 2pi
    }

    /**
     * Turns turret to the robot-relative angle
     * @param angle in radians
     */
    public void turnToRobotAngle(double angle) {
        double targetPos = angle / (2 * Math.PI);
        turret.setPosition(targetPos);
    }

    /**
     * @return Turret's robot-relative angle (in radians)
     */
    public double getTurretRobotAngle() {
        return turret.getPosition() * 2 * Math.PI;
    }

    /**
     * @return Turret's pose on the robot based on its position and current heading
     */
    public Pose2d getPoseRobotTurret() {
        return new Pose2d(Robot.Constants.turretPos,getTurretRobotAngle());
    }
}
