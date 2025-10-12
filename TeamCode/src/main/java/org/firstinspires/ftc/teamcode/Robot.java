package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// Full Robot: drive, flywheel, turret, hood, feeder, intake, camera
@Config
public class Robot {
    // TODO: Red/blue pose mirroring
    //  Lookup table functionality
    //  Turret position mapped correctly to robot pose etc.
    //  Telemetry
    //  Some way to carry the information over to teleop, not have to reinitialize (singleton later)

    AprilTagMecanumDrive drive; // Not used right now
    AprilDrive drive2; // Drive base of the robot including motors, odometry, pinpoint, and camera (capable of hybrid localization)
    Intake intake; // Motor subsystem that runs continuously to spin the intake axle
    Feeder feeder; // Motor subsystem that runs occasionally to move the ball up the elevator into the flywheel
    Flywheel flywheel; // Motor subsystem that spins to certain target RPM throughout the match
    Turret turret; // Servo subsystem that turns to any robot-relative angle
    Hood hood; // Servo subsystem that raises or lowers the hood anywhere from low to high degrees
    AprilTagLocalization2 camera; // Camera subsystem that is used in AprilDrive and Obelisk detection

    // Enum that stores the color of alliance, accessible globally
    public enum Color {
        RED,
        BLUE
    }
    public final Color color; // Create an instance of the enum to initialize later


    // Initialize subsystems

    /**
     * @param hardwareMap
     * @param initialPose robot's starting pose on the field
     * @param color alliance color
     */
    public Robot(HardwareMap hardwareMap, Pose2d initialPose, Color color){
        this.color = color; // Pose mirroring can occur depending on color
        switch (color){
            case RED:
                Constants.goalPos = new Vector2d(-58.3727,55.6425);
            case BLUE:
                Constants.goalPos = new Vector2d(-58.3727,-55.6425);
                initialPose = mirrorPose(initialPose);
        }
        camera = new AprilTagLocalization2(hardwareMap);
//        drive = new AprilTagMecanumDrive(hardwareMap, initialPose, camera);
        drive2 = new AprilDrive(hardwareMap, initialPose);
        intake = new Intake(hardwareMap);
        feeder = new Feeder(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        turret = new Turret(hardwareMap);
        hood = new Hood(hardwareMap);
    }

    /**
     * Mecanum drive with AprilTag localization
     */
    private class AprilDrive extends MecanumDrive { // If this works, preferred over AprilTagMecanumDrive
        // Localizer functions reset with AprilTags
        public AprilDrive(HardwareMap hardwareMap, Pose2d initialPose){
            // Just use the parent constructor (MecanumDrive), we only are creating one method
            super(hardwareMap,initialPose);
        }
        // TODO: maybe want to return a boolean of successful relocalization instead of the pose, which is accessible by other means. Then we can keep trying to relocalize until we get a success.

        /**
         * This function uses AprilTag detection to determine robot's field-relative pose, updates the localizer with this pose (unless no detection), and returns the pose.
         * We should use this every ~10 seconds, use the goal AprilTag to re-define the robot's pose relative to the field.
         */
        @SuppressLint("DefaultLocale")
        public Pose2d relocalize(Telemetry telemetry) {
            PoseVelocity2d vel = super.updatePoseEstimate(); // Update the pinpoint velocity estimate as normal

            // Don't relocalize if the robot is moving too fast. The motion blur will cause some problems
            if (vel.linearVel.norm() > 1.0 || Math.toDegrees(vel.angVel) > 1.0) {
                return localizer.getPose();
            }

            // Get the pose of the turret relative to the field using the goal Apriltag detected by camera.
            Pose2d poseWorldTurret = camera.update(telemetry);
//            pose = new Pose2d(pose.position.minus(Constants.turretPos),pose.heading.toDouble()-turret.getTurretRobotAngle());

            // If no pose was found, default to the pinpoint localizer relative pose, end function.
            if (poseWorldTurret == null){
                return localizer.getPose();
            }
            // Pose of the turret relative to field using (AprilTag detection + 90 degrees)
            poseWorldTurret = new Pose2d(poseWorldTurret.position,poseWorldTurret.heading.toDouble() + Math.PI/2);

            // Pose multiplication: pWR = pWT * pRT^-1. Transforming the turret pose into the robot pose.
            // Uses the pose of the turret and turret's position on robot to get robot's field-relative pose
            Pose2d poseWorldRobot = poseWorldTurret.times(turret.getPoseRobotTurret().inverse());
            localizer.setPose(poseWorldRobot); // Reset the localizer pose to the current field-relative pose.

            // Telemetry lines
            telemetry.addLine(String.format("Pose XY %6.1f %6.1f  (inch)",
                    poseWorldRobot.position.x,
                    poseWorldRobot.position.y));
            telemetry.addLine(String.format("Pose Heading %6.1f  (rad)",
                    poseWorldRobot.heading.toDouble()
            ));
            return poseWorldRobot; // Return the pose for our needs.
        }
    }

    // Shooting system state enum
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    private LaunchState launchState;

    // Drive with either pinpoint or AprilTag localization
    private enum DriveState {
        RELATIVE,
        ABSOLUTE // Drive with AprilTag localization for absolute position on field
    }
    private DriveState driveState;

    ElapsedTime feederTimer; // Times the shooting process
    ElapsedTime aprilTimer; // Times the AprilTag localization process

    /**
     * Initialize and set motors, shooter, timers
     */
    public void initTeleop(Telemetry telemetry) {
        launchState = LaunchState.IDLE;
        driveState = DriveState.RELATIVE;

        feederTimer = new ElapsedTime();
        aprilTimer = new ElapsedTime();

        Constants.drivePower = 0.5; // Power on motors

        telemetry.addData("Status", "Initialized");
//        telemetry.update();
    }

    // Robot constants
    @Config
    public static class Constants {
        // SHOOTER SYSTEM CONSTANTS
        public final static Vector2d turretPos = new Vector2d(0,0); // Turret position on robot
        public static double flywheelPower = 3;
        public final static double deltaH = 30; // Height difference between shooter and goal
        public static Vector2d goalPos = new Vector2d(-58.3727,55.6425); // Goal location
        public final static Pose2d poseTurretCamera = new Pose2d(0, 3, 0); // Pose of camera compared to turret
        public static double p = 300, i = 0, d = 0, f = 10; // PIDF constants
        public final static double feederPower = 1.0; // Power of feeder to the elevator
        public final static double intakePower = 1.0; // Intake rollers
        public final static double outtakePower = -1.0; // Intake rollers
        public final static double FEED_TIME_SECONDS = 0.20; // The feeder servos run this long when a shot is requested.
        public final static double FLYWHEEL_STOP_SPEED = 0.0; // We send this power to the servos when we want them to stop.
        public final static double FLYWHEEL_FULL_SPEED = 1.0;
        public final static double FLYWHEEL_TARGET_VELOCITY = 1125;
        public final static double FLYWHEEL_MIN_VELOCITY = 1075;
        // HOOD CONSTANTS
        public final static double hoodLowAngle = 87 * Math.PI / 180; // Ball trajectory angle from horizontal (rad)
        public final static double hoodHighAngle = 50 * Math.PI / 180; // Ball trajectory angle from horizontal
        public final static double hoodScale0 = 0.27;
        public final static double hoodScale1 = 0.85;
        // TURRET CONSTANTS
        public final static double turretHighAngle = Math.PI / 2; // In rad, pos = 1 (90 deg)
        public final static double turretLowAngle = -11 * Math.PI / 6; // In rad (-330 deg)
        public final static double turretScale0 = 0;
        public final static double turretScale1 = 1;
        public static double drivePower = 1.0;
    }

    /**
     * Drives the robot field-centric
     */
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

    /**
     * Calculates and sets hood angle and flywheel RPM.
     * Includes shooter state manager.
     * @param shotRequested: Boolean to start shooter state machine
     */
    public void updateShooter(boolean shotRequested, Telemetry telemetry) {
        // TODO: Replace these with LUT values
        // Assume we have: Vector2d goalPosition
        Pose2d pose = drive2.localizer.getPose(); // Robot pose
        Vector2d goalVector = Constants.goalPos.minus(pose.position); // Goal vector relative to robot pos and heading

        double p = 0.9; // Fraction of time along trajectory from ground to ground
        double g = 386.22; // Gravity acceleration (in/s^2)

        double h = Constants.deltaH; // Height difference from shooter to goal
        double d = goalVector.norm(); // Horizontal distance from base of robot to goal

        double time_total = Math.sqrt(2 * h / (p * g * (1 - p))); // Ball trajectory time from ground to ground
        double theta = Math.atan(h / (d * (1 - p))); // Ball launch - angle of elevation
        double v = d / (p * time_total * Math.cos(theta)); // Ball launch speed
//        double absoluteAngleToGoal = /*Math.PI + */Constants.goalPos.minus(pose.position).angleCast().toDouble();
        double turretAngle = goalVector.angleCast().toDouble() - pose.heading.toDouble(); // Turret angle to set to face make turret face the goal

        /*
         Calculate launch angle theta
        double theta = Math.atan(deltaH*(proportionAlongTraj+1) / (proportionAlongTraj * d));

         Calculate time of flight t_f
        double t_f = Math.sqrt(2 * deltaH / (g*proportionAlongTraj));

         Calculate initial speed v
        double v = d / (t_f * Math.cos(theta));

         Convert v (speed) to rad/s (example calibration: v = wheelRadius * rad/s)
         */

        double wheelRadius = 1.89; // inches, for example
        /*
        double wheelCircumference = Math.PI * wheelDiameter;
        double change = 0;
        if (up) change += 0.001;
        if (down) change -= 0.001;
        StarterRobot.Constants.flywheelPower += change;
         */
        double radps = v / wheelRadius * Constants.flywheelPower; // Radians per second of flywheel power

        // TURN TURRET, SET FLYWHEEL RPM, AND TURN HOOD
        turret.turnToRobotAngle(turretAngle);
        flywheel.spinTo(radps * 28 / Math.PI / 2);
        hood.turnToAngle(theta);

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

        // Telemetry lines
        if (hood.commandedOutsideRange()) telemetry.addLine("WARNING: hood commanded out of its range! Auto set to 0 or 1.");
        telemetry.addData("Flywheel power scale factor", Constants.flywheelPower);
        telemetry.addData("Launch State", launchState);
        telemetry.addLine("GoalVector (inch): " + goalVector.x + " " + goalVector.y);
        telemetry.addData("Distance to goal (inch)", d);
        telemetry.addData("Turret target angle (deg)", turretAngle * 180 / Math.PI);
        telemetry.addData("Turret actual angle (deg)", turret.getTurretRobotAngle() * 180 / Math.PI);
        telemetry.addData("Turret pos", turret.turret.getPosition());
        telemetry.addData("Hood target angle (deg)", theta * 180 / Math.PI);
        telemetry.addData("Hood actual angle (deg)", hood.getAngle());
        telemetry.addData("Hood pos", hood.HOOD.getPosition());
        telemetry.addData("TargetVel (tick/s)", radps * 28 / Math.PI / 2);
        telemetry.addData("TargetVel (rad/s)", radps);
        telemetry.addData("Flywheel speed (tick/s)", flywheel.getVel());
        telemetry.addData("Flywheel speed (rad/s)", flywheel.getVel() * 2 * Math.PI / 28);
//        telemetry.update();
    }

    /**
     * Returns field-relative robot pose (calculated using turret pose), or returns Pinpoint-recorded
     * pose if no AprilTag detections. Also displays pose information on telemetry.
     */
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

    public Pose2d mirrorPose(Pose2d pose){
        return new Pose2d(new Vector2d(pose.position.x, -pose.position.y), new Rotation2d(pose.heading.real, -pose.heading.imag));
    }
//    public Action buildTrajectory(TrajectoryActionBuilder tab){
//        if (color.equals(Color.BLUE)){
//            //not sure if mirroring poses in the tab is possible
//        }
//        return tab.build();
//    }
}