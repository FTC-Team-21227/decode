package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Robot {
    //add:
    // Red/blue pose mirroring
    // Lookup table functionality
    // Turret position mapped correctly to robot pose etc.
    // Telemetry
    // Some way to carry the information over to teleop, not have to reinitialize (singleton later)

    AprilTagMecanumDrive drive; //not used right now
    AprilDrive drive2; //the drive base of the robot including motors, odometry, pinpoint, and camera, capable of hybrid localization
    Intake intake; //the motor subsystem that runs throughout the match to spin the intake axle
    Feeder feeder; //the motor subsystem that runs occasionally to move the ball up the elevator into the flywheel
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
    public Robot(HardwareMap hardwareMap, Pose2d initialPose, Color color){
        this.color = color; //pose mirroring can occur depending on color
        camera = new AprilTagLocalization2(hardwareMap);
//        drive = new AprilTagMecanumDrive(hardwareMap, initialPose, camera);
        drive2 = new AprilDrive(hardwareMap, initialPose);
        intake = new Intake(hardwareMap);
        feeder = new Feeder(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
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
        driveState = DriveState.RELATIVE;

        feederTimer = new ElapsedTime();
        aprilTimer = new ElapsedTime();

        switch (color){
            case RED:
                Constants.goalPos = new Vector2d(-58.3727,55.6425);
            case BLUE:
                Constants.goalPos = new Vector2d(58.3727,55.6425);
                drive2.localizer.setPose(mirrorPose(drive2.localizer.getPose()));
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    //constants
    @Config
    public static final class Constants{
        public final static Vector2d turretPos = new Vector2d(0,-5);
        public final static double deltaH = 22;
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
    }


    public void driveFieldCentric(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                drive2.localizer.getPose().heading.toDouble());

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);
        drive2.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -newForward,
                        -newRight
                ),
                -rotate
        ));
    }

    public void updateShooter(boolean shotRequested, Telemetry telemetry) {
        //replace these with LUT values
        // Assume we have: Vector2d goalPosition
        Vector2d goalVector = Constants.goalPos.minus(drive2.localizer.getPose().position);
        double absoluteAngleToGoal = Math.atan2(goalVector.y, goalVector.x);
        double turretAngle = absoluteAngleToGoal - drive2.localizer.getPose().heading.toDouble(); // Relative to robot's heading

        turret.turnToRobotAngle(turretAngle);

        // Given d (horizontal distance) and deltaH (height difference)
        double g = 386.22; // in/s^2

        double deltaH = Constants.deltaH;
        double d = goalVector.norm();
// Calculate launch angle theta
        double theta = Math.atan(7 * deltaH / (3 * d));

// Calculate time of flight t_f
        double t_f = Math.sqrt(8 * deltaH / (3 * g));

// Calculate initial speed v
        double v = d / (t_f * Math.cos(theta));

// Convert v to RPM (example calibration: v = wheelCircumference * RPM / 60)
        double wheelDiameter = 3.78; // inches, for example
        double wheelCircumference = Math.PI * wheelDiameter;
        double rpm = (v * 60) / wheelCircumference; // RPM

// Set hood angle to theta (convert to servo position)
        hood.turnToAngle(theta);

// Set flywheel RPM
        flywheel.spinTo(rpm);
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
                if (flywheel.getVel() > rpm-50) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                //if time delay enough
                feeder.rollIn();
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > Constants.FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    feeder.stop();
                }
                break;
        }
        telemetry.addData("State", launchState);
        telemetry.addData("motorSpeed", flywheel.getVel());
        telemetry.addData("targetSpeed", rpm);
        telemetry.update();
    }

    public void updateLocalizer(Telemetry telemetry){
        switch (driveState){
            case RELATIVE:
                drive2.updatePoseEstimate();
                if (aprilTimer.seconds() > 10){
                    driveState = DriveState.ABSOLUTE;
                }
                break;
            case ABSOLUTE:
                drive2.relocalize(telemetry);
                aprilTimer.reset();
                driveState = DriveState.RELATIVE;
                break;
        }
    }

    public void controlIntake(boolean in, boolean out, boolean stop){
        if (in) intake.intake();
        else if (out) intake.outtake();
        else if (stop) intake.stop();
    }
    //Lookup table (lut)
    public int[][][] power = {{{1}}};
    public int[][][] angle = {{{1}}};
    public int[] lookUp(Vector2d pos, Vector2d vel){

        return new int[]{power[0][0][0],angle[0][0][0]};
    }

    //misc functions
    public Pose2d mirrorPose(Pose2d pose){
        return pose;
    }
//    public Action buildTrajectory(TrajectoryActionBuilder tab){
//        if (color.equals(Color.BLUE)){
//            //not sure if mirroring poses in the tab is possible
//        }
//        return tab.build();
//    }
}