package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

@Config
public class configvalues {
    public final static Vector2d turretPos = new Vector2d(0,0);
    public static double flywheelPower = 1;
    public final static double deltaH = 30;
    public static Vector2d goalPos = new Vector2d(-58.3727,55.6425);
    public final static Pose2d poseTurretCamera = new Pose2d(0, 3, 0);
    public static double p = 300, i = 0, d = 0, f = 10;

    public final static double feederPower = 1.0;
    public final static double intakePower = 1.0;
    public final static double outtakePower = -1.0;
    public final static double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    public final static double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    public final static double FULL_SPEED = 1.0;
    public final static double LAUNCHER_TARGET_VELOCITY = 1125;
    public final static double LAUNCHER_MIN_VELOCITY = 1075;
    // HOOD CONSTANTS
    public final static double hoodLowAngle = 87; // the traj angle from horizonatla //0;
    public final static double hoodHighAngle = 50; //the traj angle from horizontal 55; // Highest actual degree is 41
    public final static double hoodScale0 = 0.27;
    public final static double hoodScale1 = 1;
    // TURRET CONSTANTS
    public final static double turretHighAngle = Math.PI/2; // In rad
    public final static double turretLowAngle = -11*Math.PI/6; // In rad (= -330 deg)
    public final static double turretScale0 = 0;
    public final static double turretScale1 = 1;
    public static double drivePower = 1.0;
}
