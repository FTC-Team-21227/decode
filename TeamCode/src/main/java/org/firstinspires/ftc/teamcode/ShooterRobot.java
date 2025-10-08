package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ShooterRobot {
    //add:
    // Red/blue pose mirroring
    // Lookup table functionality
    // Turret position mapped correctly to robot pose etc.
    // Telemetry
    // Some way to carry the information over to teleop, not have to reinitialize (singleton later)

    Flywheel flywheel; //the motor subsystem that spins to certain target RPM throughout the match
    Turret turret; //the 2-servo subsystem that turns to any robot-relative angle
    Hood hood; //the servo subsystem that raises or lowers the hood anywhere from 0 to 90 degrees
    AprilTagLocalization2 camera; //the camera subsystem that is used in AprilDrive and Obelisk detection
    //enum that stores the color of the robot, accessible globally

    public enum Color {
        RED,
        BLUE
    }
    public final Color color; //create an instance of the enum to initialize later


    //initialize subsystems
    public ShooterRobot(HardwareMap hardwareMap, Pose2d initialPose, Color color){
        this.color = color; //pose mirroring can occur depending on color
//        switch (color){
//            case RED:
//                Constants.goalPos = new Vector2d(-58.3727,55.6425);
//            case BLUE:
//                Constants.goalPos = new Vector2d(-58.3727,-55.6425);
//        }
        this.pose = initialPose;
        camera = new AprilTagLocalization2(hardwareMap);
//        drive = new AprilTagMecanumDrive(hardwareMap, initialPose, camera);
        flywheel = new Flywheel(hardwareMap);
        flywheel.FLYWHEEL.setDirection(DcMotorSimple.Direction.REVERSE);
        turret = new Turret(hardwareMap);
        hood = new Hood(hardwareMap);
    }


    private class AprilDrive extends MecanumDrive{ //if this works, preferred over AprilTagMecanumDrive
        //localizer functions reset with apriltags
        public AprilDrive(HardwareMap hardwareMap, Pose2d initialPose){ //just use the parent constructor, we only are creating one method
            super(hardwareMap,initialPose);
        }
        //relocalize method: every 10 ish seconds, use the goal AprilTag to re-define the robot's pose relative to the field.
        //maybe want to return a boolean of successful relocalization instead of the pose, which is accessible by other means. Then we can keep trying to relocalize until we get a success.
        public Pose2d relocalize(Telemetry telemetry) {
            PoseVelocity2d vel = super.updatePoseEstimate(); //update the pinpoint velocity estimate as normal
            //don't relocalize if the robot is moving too fast. The motion blur will cause some problems
            if (vel.linearVel.norm() > 1.0 || Math.toDegrees(vel.angVel) > 1.0) {
                return localizer.getPose();
            }
            Pose2d poseWorldTurret = camera.update(telemetry); //get the pose of the turret relative to the field using the Apriltag.
//            pose = new Pose2d(pose.position.minus(Constants.turretPos),pose.heading.toDouble()-turret.getTurretRobotAngle());
            //if no pose was found, default to the pinpoint localizer relative pose.
            if (poseWorldTurret == null){
                return localizer.getPose();
            }
            //Pose multiplication: pWR = pWT * pRT^-1. Transforming the turret pose into the robot pose.
            Pose2d poseWorldRobot = poseWorldTurret.times(turret.getPoseRobotTurret().inverse());
            localizer.setPose(poseWorldRobot); //reset the localizer pose to the current field-relative pose.
            return poseWorldRobot; //return the pose for our needs.
        }
    }
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    private LaunchState launchState;
    private enum DriveState {
        RELATIVE,
        ABSOLUTE,
    }
    private DriveState driveState;

    ElapsedTime feederTimer;
    ElapsedTime aprilTimer;


    public void initTeleop(Telemetry telemetry) {
        launchState = LaunchState.IDLE;
        driveState = DriveState.ABSOLUTE;

        feederTimer = new ElapsedTime();
        aprilTimer = new ElapsedTime();

        Constants.drivePower = 0.5;

        telemetry.addData("Status", "Initialized");
//        telemetry.update();
    }

    //constants
    @Config
    public static final class Constants{
        public final static Vector2d turretPos = new Vector2d(0,0);
        public static double flywheelPower = 2.315;
        public final static double deltaH = 30;
        public static Vector2d goalPos = new Vector2d(-58.3727,55.6425);
        public final static Pose2d poseTurretCamera = new Pose2d(0, 3, 0);
        public final static double p = 300, i = 0, d = 0, f = 10;

        public final static double feederPower = 1.0;
        public final static double intakePower = 1.0;
        public final static double outtakePower = -1.0;
        public final static double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
        public final static double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
        public final static double FULL_SPEED = 1.0;
        public final static double LAUNCHER_TARGET_VELOCITY = 1125;
        public final static double LAUNCHER_MIN_VELOCITY = 1075;
        // HOOD CONSTANTS
        public final static double hoodLowAngle = 0;
        public final static double hoodHighAngle = 55; // Highest actual degree is 41
        public final static double hoodScale0 = 0.27;
        public final static double hoodScale1 = 1;
        // TURRET CONSTANTS
        public final static double turretScale0 = 0;
        public final static double turretScale1 = 1;
        public final static double turretLeftAngle = 0; // In degrees
        public final static double turretRightAngle = 360; // In degrees
        public static double drivePower = 1.0;
    }

    Pose2d pose;
    public void updateShooter(boolean shotRequested, Telemetry telemetry) {
        //replace these with LUT values
        // Assume we have: Vector2d goalPosition
        Vector2d goalVector = Constants.goalPos.minus(pose.position);

        double p = 0.75; //fraction of time along trajectory from ground to ground
        double g = 386.22; // Gravity (in/s^2)

        double h = StarterRobot.Constants.deltaH; // Height difference from shooter to goal
        double d = goalVector.norm(); // Horizontal distance

        double t_tot = Math.sqrt(2*h / (p*g*(1-p))); //ball trajectory time from ground to ground
        double theta = Math.atan(h / (d*(1-p))); //ball launch angle of elevation
        double v = d / (p * t_tot * Math.cos(theta)); //ball launch speed
//        double absoluteAngleToGoal = /*Math.PI + */Constants.goalPos.minus(pose.position).angleCast().toDouble();
        double turretAngle = goalVector.angleCast().toDouble() - pose.heading.toDouble() - Math.PI; // Relative to robot's heading



        // Calculate launch angle theta
//        double theta = Math.atan(deltaH*(proportionAlongTraj+1) / (proportionAlongTraj * d));

        // Calculate time of flight t_f
//        double t_f = Math.sqrt(2 * deltaH / (g*proportionAlongTraj));

        // Calculate initial speed v
//        double v = d / (t_f * Math.cos(theta));

        // Convert v (speed) to rad/s (example calibration: v = wheelRadius * rad/s)
        double wheelRadius = 1.89; // inches, for example
//        double wheelCircumference = Math.PI * wheelDiameter;
//        double change = 0;
//        if (up) change += 0.001;
//        if (down) change -= 0.001;
//        StarterRobot.Constants.flywheelPower += change;
        double radps = v / wheelRadius * ShooterRobot.Constants.flywheelPower; // RPM
        // Set hood angle to theta (convert to servo position)
//        hood.turnToAngle(theta);

        telemetry.addData("turret turn to", turretAngle*180/Math.PI);
        telemetry.addData("flywheel vel", radps*28/Math.PI/2);
        telemetry.addData("hood turn to", theta*180/Math.PI);
        turret.turnToRobotAngle(turretAngle);
//        // Set flywheel RPM
        flywheel.spinTo(radps*28/Math.PI/2);
//// Set hood angle to theta (convert to servo position)
        hood.turnToAngle(theta*180/Math.PI);

// Set flywheel RPM
//        flywheel.spinTo(rpm);
        //-/-///
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                    feederTimer.reset();
                }
                break;
            case SPIN_UP:
//                flywheel.spinTo(Constants.LAUNCHER_TARGET_VELOCITY);
                if (flywheel.getVel() > radps*28/Math.PI/2-50) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                //if time delay enough
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > Constants.FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                }
                break;
        }
        telemetry.addData("flywheel power scale factor", StarterRobot.Constants.flywheelPower);
        telemetry.addData("State", launchState);
        telemetry.addLine("goalVector: " + goalVector.x+" "+goalVector.y);
        telemetry.addData("distance to goal", d);
        telemetry.addData("hood theta", theta*180/Math.PI);
        telemetry.addData("targetVel (rad/s)", radps);
        telemetry.addData("motorSpeed", flywheel.getVel()*2*Math.PI/28);
//        telemetry.update();
    }

    @SuppressLint("DefaultLocale")
    public void updateLocalizer(Telemetry telemetry){
        switch (driveState){
            case ABSOLUTE:
                Pose2d poseWorldTurret = camera.update(telemetry);
                if (poseWorldTurret == null){
//                    pose = new Pose2d(pose.position, angle);
                    return;
                }
                poseWorldTurret = new Pose2d(poseWorldTurret.position,poseWorldTurret.heading.toDouble()-Math.PI/2);
//            pose = new Pose2d(pose.position.minus(Constants.turretPos),pose.heading.toDouble()-turret.getTurretRobotAngle());
                telemetry.addLine("ROBOT RELOCALIZATION POSES");
                telemetry.addLine(String.format("XY %6.1f %6.1f  (inch)",
                        poseWorldTurret.position.x,
                        poseWorldTurret.position.y));
                telemetry.addLine(String.format("Y %6.1f   (rad)",
                        poseWorldTurret.heading.toDouble()
                ));
                pose = poseWorldTurret.times(new Pose2d(StarterRobot.Constants.turretPos,0).inverse());
                telemetry.addLine(String.format("Pose XY %6.1f %6.1f  (inch)",
                        pose.position.x,
                        pose.position.y));
                telemetry.addLine(String.format("Pose Heading %6.1f  (rad)",
                        pose.heading.toDouble()
                ));
//                initialHeading = pose.heading.toDouble() - angle;
//                telemetry.update();
//                return pose;
//                driveState = DriveState.RELATIVE;
        }
//        return pose;
    }
}
