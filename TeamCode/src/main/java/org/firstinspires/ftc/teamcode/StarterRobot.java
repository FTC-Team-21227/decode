package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class StarterRobot {
    Feeders feeders;
    Flywheel flywheel;
    Hood hood;
    AprilTagLocalization2 camera;
    private DcMotor W_BL;
    private DcMotor W_BR;
    private DcMotor W_FR;
    private DcMotor W_FL;
    IMU imu;

    //initialize subsystems
    public StarterRobot(HardwareMap hardwareMap){
        feeders = new Feeders(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        camera = new AprilTagLocalization2(hardwareMap);
//        hood = new Hood(hardwareMap);
        W_BL = hardwareMap.get(DcMotor.class, "W_BL");
        W_BR = hardwareMap.get(DcMotor.class, "W_BR");
        W_FR = hardwareMap.get(DcMotor.class, "W_FR");
        W_FL = hardwareMap.get(DcMotor.class, "W_FL");
        imu = hardwareMap.get(IMU.class, "imu");
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

    double initialHeading;

    public void initTeleop(Telemetry telemetry) {
        W_FR.setDirection(DcMotor.Direction.FORWARD);
        W_FL.setDirection(DcMotor.Direction.REVERSE);
        W_BR.setDirection(DcMotor.Direction.FORWARD);
        W_BL.setDirection(DcMotor.Direction.REVERSE);
        W_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W_FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        W_FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        W_BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        W_BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        W_FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        W_FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        W_BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        W_BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();

        launchState = LaunchState.IDLE;
        driveState = DriveState.ABSOLUTE;

        feederTimer = new ElapsedTime();
        aprilTimer = new ElapsedTime();

        pose = new Pose2d(0,0,0);

        initialHeading = 0;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    //constants
    @Config
    public static class Constants{
        public final static Vector2d turretPos = new Vector2d(0,0);
        public static double flywheelPower = 2.315;
        public final static double deltaH = 30;
        public final static Vector2d goalPos = new Vector2d(-58.3727,55.6425);
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
        public final static double hoodLowAngle = 0;
        public final static double hoodHighAngle = 90;
        public final static double hoodScale0 = 0;
        public final static double hoodScale1 = 1;
        public final static double turretLeftScale0 = 0;
        public final static double turretLeftScale1 = 1;
        public final static double turretRightScale0 = 0;
        public final static double turretRightScale1 = 1;
        public final static double drivePower = 0.5;
    }

    Vector2d goalVector;
    double absoluteAngleToGoal;
    Pose2d pose;

    // Thanks to FTC16072 for sharing this code!!
    public void driveFieldCentric(double forward, double right, double rotate, Telemetry telemetry) {
        double Angle_Difference;
        absoluteAngleToGoal = Math.PI + Constants.goalPos.minus(pose.position).angleCast().toDouble();
        double Heading_Angle = pose.heading.toDouble();

        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                pose.heading.toDouble());

        // Third, convert back to cartesian
//        double newForward = r * Math.sin(theta);
//        double newRight = r * Math.cos(theta);
        double Motor_FWD_input = forward;
        double Motor_Side_input = -right;
        double newForward = (Math.cos(Heading_Angle / 180 * Math.PI) * Motor_FWD_input - Math.sin(Heading_Angle / 180 * Math.PI) * Motor_Side_input) ;
        double newRight = (Math.cos(Heading_Angle / 180 * Math.PI) * Motor_Side_input + Math.sin(Heading_Angle / 180 * Math.PI) * Motor_FWD_input) / 0.7736350635 ; //*1.5

        double imu_rotation = 0;
        double Targeting_Angle = Heading_Angle;
        //AprilTag angle locking: when the driver isn't turning the robot, lock the robot's heading onto the apriltag.
        if (Math.abs(rotate) <= -0.0) {
            Targeting_Angle = absoluteAngleToGoal; // INSERT YAW ANGLE
            Angle_Difference = Heading_Angle - Targeting_Angle; // <= This is what we're using
            if (Angle_Difference > Math.PI) {
                Angle_Difference = Angle_Difference - Math.PI*2;
            } else if (Angle_Difference < -Math.PI) {
                Angle_Difference = Angle_Difference + Math.PI*2;
            }
            if (Math.abs(Angle_Difference) < 2* Math.PI/180) {
                imu_rotation = 0;
            }
            //FOR PROPORTIONAL ANGLE CONTROL: CHANGE THESE TO NONZERO
//            else if (Angle_Difference >= Math.PI/180) {
//                imu_rotation = (Angle_Difference * 0.05 * 180/Math.PI);
//            }
            else {
                telemetry.addLine("correcting");
                imu_rotation = (Angle_Difference * 0.05 * 180/Math.PI);
            }
        }
        double Motor_Rotation_power = rotate * 0.35 + imu_rotation; //0.7 //0.5
        double Motor_power_BL = -(((newForward - newRight) - Motor_Rotation_power) * Constants.drivePower);
        double Motor_power_BR = -((newForward + newRight + Motor_Rotation_power) * Constants.drivePower);
        double Motor_power_FL = -(((newForward + newRight) - Motor_Rotation_power) * Constants.drivePower);
        double Motor_power_FR = -(((newForward - newRight) + Motor_Rotation_power) * Constants.drivePower);
        double m = Math.max(Math.max(Math.abs(Motor_power_BL),Math.abs(Motor_power_BR)),Math.max(Math.abs(Motor_power_FL),Math.abs(Motor_power_FR)));
        if (m > 1){
            Motor_power_BL /= m;
            Motor_power_BR /= m;
            Motor_power_FL /= m;
            Motor_power_FR /= m;
        }
        W_BL.setPower(Motor_power_BL);
        W_BR.setPower(Motor_power_BR);
        W_FR.setPower(Motor_power_FR);
        W_FL.setPower(Motor_power_FL);
        telemetry.addData("Rotation Power", Motor_Rotation_power);
        telemetry.addData("Heading", Heading_Angle);
        telemetry.addData("Targeting Angle", Targeting_Angle);
        telemetry.addData("IMU_Rotation Power", imu_rotation);    }

    double radps = 300;
    //queuer/state machine

    /**
     * Sets flywheel speed based flywheel power constant and wheel radius. Manges shooter states.
     * @param shotRequested button to start flywheel
     * @param up button to increase flywheel speed
     * @param down button to decrease flywheel speed
     */
    public void updateShooter(boolean shotRequested, boolean up, boolean down, Telemetry telemetry) {
        // Replace these with LUT values
        // Assume we have: Vector2d goalPosition
        goalVector = Constants.goalPos.minus(pose.position);
//        absoluteAngleToGoal = Math.atan2(goalVector.y, goalVector.x);
        double turretAngle = absoluteAngleToGoal - pose.heading.toDouble(); // Turret angle relative to robot's heading
//        turret.turnToRobotAngle(turretAngle);
        double proportionAlongTraj = 0.75;
        double g = 386.22; // Gravity (in/s^2)

        double deltaH = StarterRobot.Constants.deltaH; // Height difference from shooter to goal
        double d = goalVector.norm(); // Horizontal distance
        // Calculate launch angle theta
        double theta = Math.atan(deltaH*(proportionAlongTraj+1) / (proportionAlongTraj * d));

        // Calculate time of flight t_f
        double t_f = Math.sqrt(2 * deltaH / (g*proportionAlongTraj));

        // Calculate initial speed v
        double v = d / (t_f * Math.cos(theta));

        // Convert v (speed) to rad/s (example calibration: v = wheelRadius * rad/s)
        double wheelRadius = 1.89; // inches, for example
//        double wheelCircumference = Math.PI * wheelDiameter;
        double change = 0;
        if (up) change += 0.001;
        if (down) change -= 0.001;
        Constants.flywheelPower += change;
        radps = v / wheelRadius * Constants.flywheelPower; // RPM
        // Set hood angle to theta (convert to servo position)
//        hood.turnToAngle(theta);

        // Set flywheel RPM
        flywheel.spinTo(radps*28/Math.PI/2);
        //-/-///
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
//                flywheel.spinTo(Constants.LAUNCHER_TARGET_VELOCITY);
                if (flywheel.getVel() > radps*28/Math.PI/2-50) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                feeders.rollIn();
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > Constants.FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    feeders.stop();
                }
                break;
        }
        telemetry.addData("flywheel power scale factor", Constants.flywheelPower);
        telemetry.addData("State", launchState);
        telemetry.addLine("goalVector: " + goalVector.x+" "+goalVector.y);
        telemetry.addData("distance to goal", d);
        telemetry.addData("hood theta", theta*180/Math.PI);
        telemetry.addData("targetVel (rad/s)", radps);
        telemetry.addData("motorSpeed", flywheel.getVel()*2*Math.PI/28);
//        telemetry.update();
    }

    @SuppressLint("DefaultLocale")
    public Pose2d updateLocalizer(Telemetry telemetry){
        switch (driveState){
            case ABSOLUTE:
                double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                Pose2d poseWorldTurret = camera.update(telemetry);
                if (poseWorldTurret == null){
                    telemetry.addData("using imu",angle+initialHeading);
                    pose = new Pose2d(pose.position, angle+initialHeading);
                    return pose;
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
                pose = poseWorldTurret.times(new Pose2d(Constants.turretPos,0).inverse());
                telemetry.addLine(String.format("Pose XY %6.1f %6.1f  (inch)",
                        pose.position.x,
                        pose.position.y));
                telemetry.addLine(String.format("Pose Heading %6.1f  (rad)",
                        pose.heading.toDouble()
                ));
                initialHeading = pose.heading.toDouble() - angle;
//                telemetry.update();
                return pose;
//                driveState = DriveState.RELATIVE;
        }
        return pose;
    }
}