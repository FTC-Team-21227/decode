package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;
// Full Robot: drive, flywheel, turret, hood, feeder, intake, camera

public class Robot {
    private static Robot instance = null;
    // TODO: Red/blue pose mirroring
    // TODO: Lookup table functionality
    AprilDrive drive2; // Drive base of the robot (motors, odometry, pinpoint, and camera)
    Intake intake; // Motor subsystem to spin the intake
    Feeder feeder; // Servo subsystem to move the ball up the elevator into the flywheel
    Flywheel flywheel; // Motor subsystem that spins to certain target RPM throughout the match
    Turret turret; // Servo subsystem that turns to any robot-relative angle
    Hood hood; // Servo subsystem that raises or lowers the hood
    AprilTagLocalization2 camera; // Camera subsystem used in AprilDrive and Obelisk detection
//    Voltage voltageSensor;
    boolean shotReqFeederType = true; // True = front feeder
    boolean lockStarted = false; // Lock shooting vals to allow manual adjustment
    boolean tapping = false;
    double turretAngle; //Turret Angle
    double radps; // Flywheel speed
    double theta; // Hood angle
    double turretAngleOffset = 0;
    double hoodAngleOffset = 0;
    double chainShotCount = 1; // How many balls to shoot consecutively
    double lastTime = 0;
    int driveSideSign = -1;
    Pose2d txWorldPinpoint = new Pose2d(0,0,Math.PI);

    // Enum that stores the alliance color, accessible globally
    public enum Color {
        RED,
        BLUE
    }
    public Color color; // Instance of the enum to initialize later

    // Enum that stores the opmode type, accessible globally
    public enum OpModeState {
        AUTO,
        AUTO_FAR,
        TELEOP
    }
    public OpModeState opModeState; // Instance of the enum to initialize later

    // Initialize subsystems
    public Robot(Pose2d initialPose, Color color){
        this.color = color; // Pose mirroring can occur depending on color
//        switch (this.color){
//            case RED:
//                Constants.goalPos = new Vector2d(-58.3727,55.6425);
//                break;
//            case BLUE:
//                Constants.goalPos = new Vector2d(-58.3727,-55.6425);
//                Constants.autoShotPose = mirrorPose(Constants.autoShotPose);
//                initialPose = mirrorPose(initialPose);
//                break;
//        }
        if (this.color==Color.RED) {
            driveSideSign = 1;
            Positions.goalPos = Constants.goalPos;
            // Where the robot will shoot from:
            Positions.autoShotPose = Constants.autoShotPose;
            Positions.autoShotPose_Far = Constants.autoShotPose_Far;
            Positions.teleShotPose = Constants.teleShotPose;
            Positions.deltaH = Constants.deltaH;
//            initialPose = mirrorPose(initialPose);
            RobotLog.d("It's Red");
        }
        else if (this.color == Color.BLUE){
            driveSideSign = 1;
            Positions.goalPos = mirrorVector(Constants.goalPos);
            // Where the robot will shoot from:
            Positions.autoShotPose = mirrorPose(Constants.autoShotPose);
            Positions.autoShotPose_Far = mirrorPose(Constants.autoShotPose_Far);
            Positions.teleShotPose = mirrorPose(Constants.teleShotPose);
            Positions.deltaH = Constants.deltaH;
            initialPose = mirrorPose(initialPose);
            RobotLog.d("It's Blue");
        }
        else {
            RobotLog.d("What...");
        }
        txWorldPinpoint = initialPose;
//        voltageSensor = new Voltage(hardwareMap.get(VoltageSensor.class,"Control Hub"));
    }

    // Create one instance of robot (singleton). NOTE: ALL DEVICES MUST BE REINITIALIZED BEFORE EVERY OPMODE, THEY ARE NOT SAVED.
    public static Robot getInstance(Pose2d initialPose, Color color){
        if (instance == null || instance.opModeState == OpModeState.TELEOP){
            RobotLog.d("making a new instance");
            instance = new Robot(initialPose, color);
        }
        else RobotLog.d("keeping the instance");
        return instance;
    }

    public static Robot startInstance(Pose2d initialPose, Color color){
        instance = new Robot(initialPose, color);
        return instance;
    }

    public static void clearInstance(){
        instance = null;
    }


    public class AprilDrive extends MecanumDrive{
        // Reset localizer functions using AprilTags
        public AprilDrive(HardwareMap hardwareMap, Pose2d initialPose, Color color){ // Using the parent constructor since we only are creating one method
            super(hardwareMap,initialPose,color);
        }

        // TODO: Maybe want to return a boolean of successful relocalization instead of the pose,
        //  which is accessible by other means (keep trying to relocalize until we get a success)
        @SuppressLint("DefaultLocale")
        public Pose2d relocalize(Telemetry telemetry) {
            PoseVelocity2d vel = super.updatePoseEstimate(); // Update the pinpoint velocity estimate as normal
            // Don't relocalize if robot moving too fast (motion blur)
            if (vel.linearVel.norm() > 1.0 || Math.toDegrees(vel.angVel) > 1.0) {
                return localizer.getPose();
            }
            Pose2d poseWorldTurret = camera.update(telemetry); // Get camera's pose (field-relative) using AprilTag
            // If no pose was found, default to the pinpoint localizer
            if (poseWorldTurret == null){
                return localizer.getPose();
            }
            // Add 90 degrees to change AprilTag heading information to same orientation as robot.
            poseWorldTurret = new Pose2d(poseWorldTurret.position,poseWorldTurret.heading.toDouble() + Math.PI/2);
            // Pose multiplication: pWR = pWT * pRT^-1. Transforming the turret pose into the robot pose.
            Pose2d poseWorldRobot = poseWorldTurret/*.times(turret.getPoseRobotTurret().inverse())*/; //poseWortTurret doesn't really exist anymore because the camera is mounted directly on the robot.
            // Reset the localizer pose to the current field-relative pose based on AprilTag reading.
            localizer.setPose(poseWorldRobot);

            // Telemetry lines
            telemetry.addLine(String.format("Pose XY %6.1f %6.1f  (inch)",
                    poseWorldRobot.position.x,
                    poseWorldRobot.position.y));
            telemetry.addLine(String.format("Pose Heading %6.1f  (rad)",
                    poseWorldRobot.heading.toDouble()
            ));
            return poseWorldRobot; // Return the pose of robot
        }
        public void drawPose(@NonNull TelemetryPacket p){
            Canvas c = p.fieldOverlay();

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());
            c.setStroke("#00FF00");
            Drawing.drawRobot(c,localizer.getPose().times(Constants.turretPos));
        }
        /**
         * Point-to-point action: drives the robot from the current pose to targetPose using a
         * simple proportional controller (position + heading) and the same wheel kinematics/feedforward
         * approach used in FollowTrajectoryAction/TurnAction.
         *
         * Usage: Actions.run(mecanumDrive.p2pAction(targetPose, 20.0, 1.0, Math.toRadians(3)));
         */
        public class P2PAction implements Action {
            private Pose2d targetPose;
            public P2PAction(){}
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // update local estimate
                //targetPose = camera.getBallPose();
                Pose2dDual<Time> targetDual = Pose2dDual.constant(targetPose, 1);  // zero velocity/accel

                PoseVelocity2d robotVel = updatePoseEstimate();

                PoseVelocity2dDual<Time> command = new HolonomicController(
                        PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                        PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
                ).compute(targetDual, localizer.getPose(), robotVel);

                MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
                double voltage = voltageSensor.getVoltage();

                final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                        PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
                double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
                double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
                double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
                double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
//                mecanumCommandWriter.write(new MecanumCommandMessage(
//                        voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
//                ));

                // Telemetry lines
                // Robot current position
                packet.put("x", localizer.getPose().position.x);
                packet.put("y", localizer.getPose().position.y);
                packet.put("heading (deg)", Math.toDegrees(localizer.getPose().heading.toDouble()));
                // Position error
                Pose2d error = targetPose.minusExp(localizer.getPose()); // Error pose based on difference between current and target pos
                packet.put("xError", error.position.x);
                packet.put("yError", error.position.y);
                packet.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));
                // Stop adjusting if error not too large
                if (error.position.sqrNorm() < 4 && Math.abs(error.heading.toDouble()) < 2){
                    leftFront.setPower(0);
                    leftBack.setPower(0);
                    rightBack.setPower(0);
                    rightFront.setPower(0);
                    return false;
                }
                // Set wheel power
                leftFront.setPower(leftFrontPower);
                leftBack.setPower(leftBackPower);
                rightBack.setPower(rightBackPower);
                rightFront.setPower(rightFrontPower);
//                Canvas c = packet.fieldOverlay();
//                drawPoseHistory(c);
//                c.setStroke("#FF9800");
//                Drawing.drawRobot(c, targetPose);
//                c.setStroke("#3F51B5");
//                Drawing.drawRobot(c, localizer.getPose());

//                started = true;
                return true;
            }
            @Override
            public void preview(Canvas c) {
                c.setStroke("#FF98007A");
                c.fillCircle(targetPose.position.x, targetPose.position.y, 2);
            }
        }
        public Action p2pAction() {
            return new P2PAction();
        }
    }
    private enum LaunchState {
        IDLE,
        SPIN_UP_FRONT,
        SPIN_UP_BACK,
        FEED_FRONT,
        FEED_BACK,
        LAUNCHING,
        FEED_DOWN,
    }
    private LaunchState launchState;

    private enum DriveState {
        RELATIVE,
        ABSOLUTE // Drive with AprilTag localization for absolute position on field
    }
    private DriveState driveState;

    ElapsedTime feederTimer;
    ElapsedTime aprilTimer; //gonna use aprilTimer to also track loop times. Sorry, not sorry.
    ElapsedTime intakeTimer; //this is so cancer


    // Initialize and set motors, shooter, timers
    public void initAuto(HardwareMap hardwareMap, Telemetry telemetry, OpModeState opModeState) {
//        if (color == Color.BLUE) {
//            txWorldPinpoint = mirrorPose(txWorldPinpoint); //new Pose2d(-57.25, 46, Math.toRadians(-50)); //must mirror it back to redness
//            RobotLog.dd("Blue initial pose", txWorldPinpoint.position.x + " " + txWorldPinpoint.position.y + " " + txWorldPinpoint.heading.toDouble());
//        }
        if (opModeState == OpModeState.AUTO_FAR){
            Positions.goalPos = handleVector(Constants.goalPos_far);
            Positions.deltaH = Constants.deltaH_far;
        }
        intake = new Intake(hardwareMap);
        camera = new AprilTagLocalization2(hardwareMap);
        drive2 = new AprilDrive(hardwareMap, txWorldPinpoint,color);
        feeder = new Feeder(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        turret = new Turret(hardwareMap);
        hood = new Hood(hardwareMap);

//        opModeState = OpModeState.AUTO;
        launchState = LaunchState.IDLE;
        driveState = DriveState.RELATIVE;

        feeder.downBL();
        feeder.downFR();

        feederTimer = new ElapsedTime();
        aprilTimer = new ElapsedTime();
        intakeTimer = new ElapsedTime();

        Positions.drivePower = Constants.drivePower;
        Positions.turretClip0 = Constants.turretClip0;
        Positions.turretClip1 = Constants.turretClip1;

        RobotLog.dd("goal pos", " " + Positions.goalPos.x + " " + Positions.goalPos.y);
        RobotLog.dd("auto shot pose", " " + Positions.autoShotPose.position.x + " " +  Positions.autoShotPose.position.y + " " + Positions.autoShotPose.heading.toDouble());
        RobotLog.dd("auto shot pose far", " " + Positions.autoShotPose_Far.position.x + " " +  Positions.autoShotPose_Far.position.y + " " + Positions.autoShotPose_Far.heading.toDouble());
        RobotLog.dd("tele shot pose", " " + Positions.teleShotPose .position.x + " " +  Positions.teleShotPose .position.y + " " + Positions.teleShotPose .heading.toDouble());
        RobotLog.dd("initial pose", " " + txWorldPinpoint.position.x + " " + txWorldPinpoint.position.y + " " + txWorldPinpoint.heading.toDouble());
        RobotLog.dd("deltaH"," "+Positions.deltaH);
        telemetry.addData("goal pos", " " + Positions.goalPos.x + " " + Positions.goalPos.y);
        telemetry.addData("auto shot pose far", " " + Positions.autoShotPose_Far.position.x + " " +  Positions.autoShotPose_Far.position.y + " " + Positions.autoShotPose_Far.heading.toDouble());
        telemetry.addData("tele shot pose", " " + Positions.teleShotPose .position.x + " " +  Positions.teleShotPose .position.y + " " + Positions.teleShotPose .heading.toDouble());
        telemetry.addData("initial pose", " " + txWorldPinpoint.position.x + " " + txWorldPinpoint.position.y + " " + txWorldPinpoint.heading.toDouble());
        telemetry.addData("auto shot pose", " " + Positions.autoShotPose.position.x + " " +  Positions.autoShotPose.position.y + " " + Positions.autoShotPose.heading.toDouble());
        telemetry.addData("deltaH"," "+Positions.deltaH);
        telemetry.addData("Status", "Initialized"); //IMPORTANT
//        telemetry.update();
    }
    // Initialize and set motors, shooter, timers
    public void initTeleop(HardwareMap hardwareMap, Telemetry telemetry) {
//        if (color == Color.RED){
//            Constants.turretScale0 = 0; //
//            Constants.turretScale1 = 1;
//
//        }
//        Constants.turretTargetRangeOffset = (Constants.turretLowAngle + Constants.turretHighAngle )/2.0;
        intake = new Intake(hardwareMap);
        camera = new AprilTagLocalization2(hardwareMap);
        drive2 = new AprilDrive(hardwareMap, txWorldPinpoint, color);
        feeder = new Feeder(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        turret = new Turret(hardwareMap);
        hood = new Hood(hardwareMap);

        opModeState = OpModeState.TELEOP;
        launchState = LaunchState.IDLE;
        driveState = DriveState.RELATIVE;

        feederTimer = new ElapsedTime();
        aprilTimer = new ElapsedTime();
        intakeTimer = new ElapsedTime();

        Positions.drivePower = Constants.drivePower_Tele;
        Positions.turretClip0 = Constants.turretClip0_tele;
        Positions.turretClip1 = Constants.turretClip1_tele;

        RobotLog.dd("goal pos", " " + Positions.goalPos.x + " " + Positions.goalPos.y);
        RobotLog.dd("auto shot pose", " " + Positions.autoShotPose.position.x + " " +  Positions.autoShotPose.position.y + " " + Positions.autoShotPose.heading.toDouble());
        RobotLog.dd("auto shot pose far", " " + Positions.autoShotPose_Far.position.x + " " +  Positions.autoShotPose_Far.position.y + " " + Positions.autoShotPose_Far.heading.toDouble());
        RobotLog.dd("tele shot pose", " " + Positions.teleShotPose .position.x + " " +  Positions.teleShotPose .position.y + " " + Positions.teleShotPose .heading.toDouble());
        RobotLog.dd("initial pose", " " + txWorldPinpoint.position.x + " " + txWorldPinpoint.position.y + " " + txWorldPinpoint.heading.toDouble());
        RobotLog.dd("deltaH"," "+Positions.deltaH);
        telemetry.addData("goal pos", " " + Positions.goalPos.x + " " + Positions.goalPos.y);
        telemetry.addData("auto shot pose far", " " + Positions.autoShotPose_Far.position.x + " " +  Positions.autoShotPose_Far.position.y + " " + Positions.autoShotPose_Far.heading.toDouble());
        telemetry.addData("tele shot pose", " " + Positions.teleShotPose .position.x + " " +  Positions.teleShotPose .position.y + " " + Positions.teleShotPose .heading.toDouble());
        telemetry.addData("initial pose", " " + txWorldPinpoint.position.x + " " + txWorldPinpoint.position.y + " " + txWorldPinpoint.heading.toDouble());
        telemetry.addData("auto shot pose", " " + Positions.autoShotPose.position.x + " " +  Positions.autoShotPose.position.y + " " + Positions.autoShotPose.heading.toDouble());
        telemetry.addData("deltaH"," "+Positions.deltaH);
        telemetry.addData("Status", "Initialized"); //IMPORTANT
//        RobotLog.d("dirve side sign"+driveSideSign);
//        telemetry.update();
    }
    public void startTeleop(){
        feeder.downBL();
        feeder.downFR();
    }
    // Constants
    public static class Constants{
        public final static Pose2d turretPos = new Pose2d(-1.512,-0.12224409,0); //1.5
        public static double flywheelPower = 2.4;
        public final static double deltaH = 39; //height of wall = 54, height of ball exit = 18 inch - 3 inch //32; //height of turret = 8.436535433
        public final static double deltaH_far = 41;
        public final static Vector2d goalPos = new Vector2d(-58.3727-5.3,55.6425+5.3); //distance to triangle center = 7.5 inch
        public final static Vector2d goalPos_far = new Vector2d(-70,55.6425+12); //distance to triangle center = 7.5 inch
        // Where the robot will shoot from:
        public final static Pose2d autoShotPose = new Pose2d(-12,15,Math.toRadians(90));
        public final static Pose2d autoShotPose_Far = new Pose2d(56,12,Math.toRadians(120));
        public final static Pose2d teleShotPose = new Pose2d(0,0,goalPos.angleCast().toDouble()+Math.PI);
        public final static double kP = 0.052, kI = 0, kD = 0.000, kF = 10 /*kF will be removed in our new version*/, kS = 0.65, kV = 0.00450;
        public final static double feederPower = 1.0;
        public final static double intakePower = 1.0;
        public final static double slowIntakePower = 0.7;
        public final static double outtakePower = 0.4;
        public final static double intakePulseTime = 0.5;
        public final static double outtakePulseTime = 0.2;
        public final static double intakeStabilizeTime = 0.3;
        public final static double feedTime = 0.5; //0.5; //probably increase to 0.6
        public final static double feedDownTime = 0.4; //less than feedUpTime probably
        public final static double spinUpTimeout = 2;
        public final static double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
        public final static double FULL_SPEED = 1.0;
        public final static double LAUNCHER_TARGET_VELOCITY = 1125;
        public final static double LAUNCHER_MIN_VELOCITY = 1075;
        // HOOD CONSTANTS
        public final static double hoodLowAngle = 65.36742754 * Math.PI/180; // the traj angle from horizonatla (rad) //75 //0;
        public final static double hoodHighAngle = 24.47717215 * Math.PI/180; //30 //50*Math.PI/180; //the traj angle from horizontal 55; // Highest actual degree is 41
        public final static double hoodScale0 = 0.15; //0.27;
        public final static double hoodScale1 = 0.83; //1; //0.85;
        // TURRET CONSTANTS
        //turret 0 = 0.48
        //oldest ones also work
        //for backwards turret, add math.pi here
        public final static double turretHighAngle = 3*Math.PI/2; //164.7*Math.PI/180; //220*Math.PI/180;; //355*Math.PI/180; // //140*Math.PI/180; // In rad, pos = 1
        public final static double turretLowAngle = Math.PI/2; //-175*Math.PI/180; //-40*Math.PI/180;; // //-208*Math.PI/180; // In rad (= old -330 deg)
//        public final static double turretTargetRangeOffset = turretHighAngle-Math.PI; //offset from (-pi,pi)
        public final static double turretTargetRangeOffset = Math.PI/2; //(turretLowAngle + turretHighAngle )/2.0; //turretHighAngle-Math.PI; //offset from (-pi,pi) to (midpoint-pi, midpoint+pi), i.e. shift midpoint from 0 to new midpoint
        public final static double turretScale0 = 0.33055555555555555; //0.218; //0; //0.25 ;//0; //0.11;
        public final static double turretScale1 = 0.7838888888888889; //0.67; //1; //0.78; //0.86; //1;
        public final static double turretClip0 = 0;
        public final static double turretClip0_tele = turretScale0;
        public final static double turretClip1 = 0.8;
        public final static double turretClip1_tele = turretScale1;
        public final static double feederScale0 = 0;
        public final static double feederScale1 = 1;
        public final static double drivePower = 1.0;
        public final static double drivePower_Tele = 1.0;
        public final static double drivePower_Slow = 0.2;
    }
    public static class Positions{ //These are mobile positions, which are mirrored depending on color. Also includes scalars that are toggled between different values. For better code readability and error proofing.
        public static Vector2d goalPos = Constants.goalPos;
        // Where the robot will shoot from:
        public static Pose2d autoShotPose = Constants.autoShotPose;
        public static Pose2d autoShotPose_Far = Constants.autoShotPose_Far;
        public static Pose2d teleShotPose = Constants.teleShotPose;
        public static double deltaH = Constants.deltaH;
        public static double drivePower = Constants.drivePower;
        public static double turretClip0 = Constants.turretClip0;
        public static double turretClip1 = Constants.turretClip1;
    }
    // Drives the robot field-centric
    public void driveFieldCentric(double forward, double right, double rotate, boolean slowMode) {
        if (slowMode){
            Positions.drivePower = Constants.drivePower_Slow;
        }
        else{
            Positions.drivePower = Constants.drivePower_Tele;
        }
        double Heading_Angle = drive2.localizer.getPose().heading.toDouble();
        double mag = Math.sqrt(forward*forward + right*right);
        double Motor_FWD_input = right * mag * driveSideSign; //forward * mag;
        double Motor_Side_input = forward * mag * driveSideSign; //-right * mag;
        double Motor_fwd_power = Math.cos(Heading_Angle) * Motor_FWD_input - Math.sin(Heading_Angle) * Motor_Side_input;
        double Motor_side_power = (Math.cos(Heading_Angle) * Motor_Side_input + Math.sin(Heading_Angle) * Motor_FWD_input) * MecanumDrive.PARAMS.inPerTick / MecanumDrive.PARAMS.lateralInPerTick; //*1.5
        double Motor_Rotation_power = rotate * 0.7/* * driveSideSign*/; //0.5
        double Motor_power_BL = -(((Motor_fwd_power - Motor_side_power) - Motor_Rotation_power) * Positions.drivePower);
        double Motor_power_BR = -(((Motor_fwd_power + Motor_side_power) + Motor_Rotation_power) * Positions.drivePower);
        double Motor_power_FL = -(((Motor_fwd_power + Motor_side_power) - Motor_Rotation_power) * Positions.drivePower);
        double Motor_power_FR = -(((Motor_fwd_power - Motor_side_power) + Motor_Rotation_power) * Positions.drivePower);
        double maxPowerMag = 1;
        maxPowerMag = Math.max(maxPowerMag, Math.abs(Motor_power_BL));
        maxPowerMag = Math.max(maxPowerMag, Math.abs(Motor_power_BR));
        maxPowerMag = Math.max(maxPowerMag, Math.abs(Motor_power_FL));
        maxPowerMag = Math.max(maxPowerMag, Math.abs(Motor_power_FR));
        drive2.leftFront.setPower(Motor_power_FL / maxPowerMag);
        drive2.rightFront.setPower(Motor_power_FR / maxPowerMag);
        drive2.leftBack.setPower(Motor_power_BL / maxPowerMag);
        drive2.rightBack.setPower(Motor_power_BR / maxPowerMag);
    }

    // Calculates and sets hood angle and flywheel RPM. Includes shooter state manager.
    public void updateShooter(boolean shotRequestedFront, boolean shotRequestedBack, boolean shotReqAlt,
                              Telemetry telemetry, boolean setPose, Pose2d setRobotPose,
                              double flywheelChange, boolean hoodUp, boolean hoodDown,
                              boolean turretLeft, boolean turretRight, boolean humanFeed) {
        // TODO: add some params where u can hardset shooter outputs - done
        // TODO: replace these with LUT values - maybe not need rn
        // Assume we have: Vector2d goalPosition
        Pose2d poseRobot = drive2.localizer.getPose(); // Robot pose

        // if setPose just became true, recompute next loop:
        if (!setPose) lockStarted = false;
        if (setPose && setRobotPose != null){
            poseRobot = setRobotPose; // Calculate shooting values on where we plan to shoot
//            RobotLog.d("Correct auto pose!");
        }
        else {
            if (opModeState == OpModeState.AUTO)
                RobotLog.d("This is auto right now, it's bad because it's calculating with the wrong pose");
        }
        Pose2d pose = poseRobot.times(Constants.turretPos); // Pose with the TURRET's position on the robot and ROBOT's heading
        Vector2d goalVector = Positions.goalPos.minus(pose.position);

        double p = 0.65; // Fraction of time along trajectory from ground to ground
        double g = 386.22; // Gravity (in/s^2)
        double deltaH = Positions.deltaH; // Height difference from shooter to goal
        double distance = goalVector.norm(); // Horizontal distance
        double goalVectorAngle = goalVector.angleCast().toDouble();

        //
        if (!setPose || !lockStarted) {
            lockStarted = true;
            try {
                double flightTime = Math.sqrt(2 * deltaH / (p * g * (1 - p))); // Ball trajectory time from ground to ground
                theta = Math.atan(deltaH / (distance * (1 - p))); // Ball launch angle of elevation
                double vel = distance / (p * flightTime * Math.cos(theta)); // Ball launch speed
                double heading = pose.heading.toDouble(); //
                turretAngle = goalVectorAngle - heading; // Angle to turn turret to (relative to robot's heading)
                double wheelRadius = 1.89;
            /*
            double wheelCircumference = Math.PI * wheelDiameter;
            double change = 0;
            if (up) change += 0.001;
            if (down) change -= 0.001;
            StarterRobot.Constants.flywheelPower += change;
             */
                // Convert vel (speed) to rad/s (example calibration: vel = wheelRadius * rad/s
                radps = vel / wheelRadius; // RPM
            }
            catch(ArithmeticException e){
                RobotLog.dd("SHOOTER CALC FAILED MATH", e.getMessage());
                lockStarted = false;
            }
        }
//        else{
//
//        }
        // Manually control hood
        double hoodChange = 0;
        if (hoodUp) hoodChange -= Math.PI/180;
        if (hoodDown) hoodChange += Math.PI/180;
        hoodAngleOffset += hoodChange;
        // Manually control turret
        double turretChange = 0;
        if (turretLeft) turretChange -= Math.PI/180;
        if (turretRight) turretChange += Math.PI/180;
        turretAngleOffset += turretChange;

        // Adjust flywheel RPM
        double delta = 0;
        // If the flywheel power change is a big number, adjust little by little
        if (Math.abs(flywheelChange) > 0.95) delta = Math.signum(flywheelChange)*0.001;
        Constants.flywheelPower += delta;

        // Spin reverse direction for human feeding (when button is held down)
        if (humanFeed){
            //maybe also keep hood low and turret at constant pos
            radps = -800 / 28.0 * Math.PI * 2;
            theta = Math.PI/2;
            turretAngle = 0;
        }
        flywheel.spinTo(radps * 28 / Math.PI / 2 * Constants.flywheelPower);
        // Set hood angle to theta (convert to servo position)
        hood.turnToAngle(theta+hoodAngleOffset);
        turret.turnToRobotAngle(turretAngle+turretAngleOffset);

        //use +-1 of requesting state

        switch (launchState) {
            case IDLE:
                if (shotReqAlt /*&& feederTimer.seconds()>Constants.feedTime*/){
                    // If 2 artifacts to shoot consecutively
                    if (chainShotCount == 2){
                        if (feederTimer.seconds() > /*Constants.feedTime + */Constants.intakePulseTime+Constants.intakeStabilizeTime) {
                            // True = front feeder, False = back feeder
                            if (shotReqFeederType) launchState = LaunchState.SPIN_UP_FRONT;
                            else launchState = LaunchState.SPIN_UP_BACK;
                            // Alternate feeders
                            shotReqFeederType = !shotReqFeederType;
                            feederTimer.reset();
                        }
                        // Stop pulse
                        else if (feederTimer.seconds() > /*Constants.feedTime +*/ Constants.intakePulseTime) intake.stop();
                        // Intake pulse to move ball to a spot
                        else intake.slowIntake();
                    }
                    else { // Only shooting one when shotReqAlt is True: normal shooting
                        if (shotReqFeederType) launchState = LaunchState.SPIN_UP_FRONT;
                        else launchState = LaunchState.SPIN_UP_BACK;
                        shotReqFeederType = !shotReqFeederType;
                        feederTimer.reset();
                    }
                }
                // Normal shooting
                else if (!shotReqAlt) chainShotCount = 1; //shotReqFeederType = true;
                if (shotRequestedFront/* && feederTimer.seconds()>Constants.feedTime*/) {// After feeding is done. change req state here too
                        launchState = LaunchState.SPIN_UP_FRONT;
                        shotReqFeederType = false; //next RB will be back
                        feederTimer.reset();
                }
                else if (shotRequestedBack/* && feederTimer.seconds()>Constants.feedTime*/) {
                        launchState = LaunchState.SPIN_UP_BACK;
                        shotReqFeederType = true;
                        feederTimer.reset();
                }
                break;
            case SPIN_UP_FRONT: // SPEED UP FLYWHEEL
                if (flywheel.getVel() > radps * 28 / Math.PI / 2 - 50 || feederTimer.seconds() > Constants.spinUpTimeout) {
                    launchState = LaunchState.FEED_FRONT;
                }
                break;
            case SPIN_UP_BACK: // SPEED UP FLYWHEEL
                if (flywheel.getVel() > radps * 28 / Math.PI / 2 - 50 || feederTimer.seconds() > Constants.spinUpTimeout) {
                    launchState = LaunchState.FEED_BACK;
                }
                break;
            case FEED_FRONT: // FEED BALL
                intake.pause();
                feeder.upFR(); // feeder starts
                feederTimer.reset(); // feeder goes down
                launchState = LaunchState.LAUNCHING;
                break;
            case FEED_BACK: // FEED BALL
                intake.pause();
                feeder.upBL();
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING: // RESET EVERYTHING
                if (feederTimer.seconds() > Constants.feedTime) {
                    launchState = LaunchState.FEED_DOWN;
                    feeder.downFR();
                    feeder.downBL();
                    feederTimer.reset();
                }
                break;
            case FEED_DOWN: // RESET EVERYTHING
                if (feederTimer.seconds() > Constants.feedDownTime) {
                    launchState = LaunchState.IDLE;
                    feederTimer.reset();
                    intake.proceed();
                    if (shotReqAlt) chainShotCount++;
                }
                break;
        }

        // TELEMETRY LINES
        if (hood.HOOD.commandedOutsideRange()) telemetry.addLine("WARNING: hood commanded out of its range! Auto set to 0 or 1.");
        if (turret.turret.commandedOutsideRange()) telemetry.addLine("WARNING: turret commanded out of its range! Auto set to 0 or 1.");
        telemetry.addData("flywheel power scale factor", Constants.flywheelPower);
        telemetry.addData("State", launchState);
        telemetry.addData("Next feeder type", shotReqFeederType);
        telemetry.addData("human feed", humanFeed); //NOT IMPORTANT
        telemetry.addData("setPose", setPose); //NOT IMPORTANT
        telemetry.addLine("robotTurretPose (inchxinchxdeg): " + pose.position.x+" "+pose.position.y+" "+pose.heading.toDouble()); //NOT IMPORTANT
        telemetry.addLine("goalVector (inchxinch): " + goalVector.x+" "+goalVector.y);
        telemetry.addLine("goalPos (inchxinch): " + Positions.goalPos.x+" "+Positions.goalPos.y); //NOT IMPORTANT
        telemetry.addData("goalVector angle (rad to deg)", Math.toDegrees(goalVectorAngle)); //NOT IMPORTANT
        telemetry.addData("distance to goal (inch)", distance); //NOT IMPORTANT
        telemetry.addData("turret angle (rad to deg)", turretAngle*180/Math.PI);
        telemetry.addData("turret angle offset (rad to deg)", turretAngleOffset*180/Math.PI);
        telemetry.addData("turret get angle (rad to deg)", turret.getTurretRobotAngle()*180/Math.PI); //NOT IMPORTANT
        telemetry.addData("turret pos", turret.turret.getPosition()); //NOT IMPORTANT
        telemetry.addData("hood theta (rad to deg)", theta*180/Math.PI);
        telemetry.addData("hood angle offset (rad to deg)", hoodAngleOffset*180/Math.PI);
        telemetry.addData("hood get angle (rad to deg)", hood.getAngle()); //NOT IMPORTANT
        telemetry.addData("hood pos", hood.HOOD.getPosition()); //NOT IMPORTANT
        telemetry.addData("targetVel (rad/s to tick/s)", radps*28/Math.PI/2);
        telemetry.addData("targetVel (rad/s)", radps); //NOT IMPORTANT
        telemetry.addData("motorSpeed (tick/s)", flywheel.getVel());
        telemetry.addData("motorSpeed (tick/s to rad/s)", flywheel.getVel()*2*Math.PI/28); //NOT IMPORTANT
        telemetry.addData("feeder fr pos", feeder.FR_FEEDER.getPosition());
        telemetry.addData("feeder bl pos", feeder.BL_FEEDER.getPosition());
        telemetry.addData("feeder seconds", feederTimer.seconds()); //NOT IMPORTANT
//        telemetry.update();
    }

    /**
     * Returns field-relative robot pose (calculated using turret pose), or returns Pinpoint-recorded
     * pose if no AprilTag detections. Also displays pose information on telemetry.
     */
    public void updateLocalizer(boolean relocalize, Telemetry telemetry){
        switch (driveState){
            case RELATIVE:
                drive2.updatePoseEstimate();
//                if (aprilTimer.seconds() > 10){
//                    driveState = DriveState.ABSOLUTE;
//                }
                if (relocalize) driveState = DriveState.ABSOLUTE;
                break;
            case ABSOLUTE:
                drive2.relocalize(telemetry);
                aprilTimer.reset();
                driveState = DriveState.RELATIVE;
                break;
        }
        double curTime = aprilTimer.milliseconds();
        telemetry.addData("loop time (ms)",curTime-lastTime);
        telemetry.addData("time since last relocalization (s)", curTime/1000.0); //IMPORTANT
        lastTime = curTime;
        telemetry.addData("x", drive2.localizer.getPose().position.x); //ALL IMPORTANT
        telemetry.addData("y", drive2.localizer.getPose().position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive2.localizer.getPose().heading.toDouble()));
    }
    public boolean setGoalTarget(){
        double x = drive2.localizer.getPose().position.x;
        boolean close = x < 10;
        if (close){
            Positions.goalPos = handleVector(Constants.goalPos);
            Positions.deltaH = Constants.deltaH;
        }
        else { //chagne to be better at far
            Positions.goalPos = handleVector(Constants.goalPos_far);
            Positions.deltaH = Constants.deltaH_far;
        }
        return close;
    }
//    public double updateVoltage(Telemetry telemetry){
//        double voltage = voltageSensor.updateVoltage();
//        telemetry.addData("volts",voltage);
//        return voltage;
//    }

    public void controlIntake(boolean in, boolean out, boolean stop, boolean tapOut, boolean inSlow){
        if (intakeTimer.seconds()>0.1){
            if (tapping) {intake.proceed(); tapping = false;}
            if (out) intake.outtake();
            else if (in) intake.intake();
            else if (inSlow) intake.slowIntake();
            else if (stop) intake.stop();
        }
        if (tapOut){
            intake.shortOuttake();
            intakeTimer.reset();
            tapping = true;
        }
    }
    public void setPose(Pose2d pose){
        txWorldPinpoint = pose;
    }

    // Lookup table (lut)
    public int[][][] power = {{{1}}};
    public int[][][] angle = {{{1}}};
    public int[] lookUp(Vector2d pos, Vector2d vel){

        return new int[]{power[0][0][0],angle[0][0][0]};
    }

    //helpers
    public Pose2d mirrorPose(Pose2d pose){
        return new Pose2d(new Vector2d(pose.position.x, -pose.position.y), new Rotation2d(pose.heading.real, -pose.heading.imag));
    }
    public Vector2d mirrorVector(Vector2d vector){
        return new Vector2d(vector.x, -vector.y);
    }
    public Pose2d handlePose(Pose2d pose){
        if (color == Color.BLUE){
            return mirrorPose(pose);
        }
        return pose;
    }
    public Vector2d handleVector(Vector2d vector){
        if (color == Color.BLUE){
            return mirrorVector(vector);
        }
        return vector;
    }

    // === Helper: Determine desired color sequence per obelisk ID ===
    public static char[] getDesiredPattern(int obeliskID) {
        switch (obeliskID) {
            case 21: return new char[]{'G','P','P'};
            case 22: return new char[]{'P','G','P'};
            case 23: return new char[]{'P','P','G'};
            default: return new char[]{'G','P','P'}; // fallback
        }
    }

    // === Helper: Compute mapping from internal queue → desired order ===
    // Slots: innermost is 0, middle is 1, one in intake is 2
    // Returns an int[] like [0, 1, 2] for firing order
    public static int[] computeFireOrder(char[] queue, char[] desired) {
        boolean[] used = new boolean[queue.length];
        int[] order = new int[desired.length];
        for (int i = 0; i < desired.length; i++) {
            for (int j = 0; j < queue.length; j++) {
                if (!used[j] && queue[j] == desired[i]) {
                    order[i] = j;
                    used[j] = true;
                    break;
                }
            }
        }
        // Mutate order to enforce physical constraints
        // --- Rule 1: Slot #2 cannot fire first ---
        if (order[0] == 2) {
            // find the first element that isn't 2 and swap
            for (int i = order.length-1; i > 0; i--) {
                if (order[i] != 2) {
                    int temp = order[0];
                    order[0] = order[i];
                    order[i] = temp;
                    break;
                }
            }
        }

        // --- Rule 2: If 1 is followed by 2, change 1 → 0 ---
        for (int i = 0; i < order.length - 1; i++) {
            if (i > 0) {
                if (order[i] == 1 && order[i + 1] == 2) {
                    order[i] = 0;
                }
                if (order[i] == 2 && order[i + 1] == 1) {
                    order[i + 1] = 0;
                }
            }
        }
        return order;
    }
    public static int[] Order = {0,1,2};
    // === Helper: Build the actual firing sequence of 3 balls based on color order ===
    public static Action shootSequence(AtomicBoolean shotReqFR, AtomicBoolean shotReqBL, AtomicBoolean slowIntakeAtomic,
                                       char[] queue, int obeliskID, int shot) {

        char[] desired = getDesiredPattern(obeliskID);
        Order = computeFireOrder(queue, desired);

        RobotLog.a("Shohtott " + shot);
        RobotLog.d("Obelisk ID: " + obeliskID);
        RobotLog.d("balls in robot: ");
        display(queue);
        RobotLog.d("balls score order: ");
        display(desired);
        RobotLog.d("feeders move order: ");
        display(Order);
        ArrayList<Action> actions = new ArrayList<>();
        for (int i = 0; i <= 2; i++) {
            int feeder = Order[i]; // Go through the firing order (eg. [0, 1, 2]) and set shot requests to true
            //TODO: modify for an intake pulse when firing slot 2. This will cause slot 1 to be displaced to slot 0. (push it in one slot)
            //The intake pulse has been changed to always occur before the second shot. This deterministically causes slot 2 to go to slot 1 and slot 1 to go to slot 0.
            if (i==1 /*feeder == 2*/) {
                actions.add(new InstantAction(() -> slowIntakeAtomic.set(true)));
                actions.add(new SleepAction(Constants.intakePulseTime));
                actions.add(new InstantAction(() -> slowIntakeAtomic.set(false)));
                actions.add(new SleepAction(Constants.intakeStabilizeTime));
            }
            // then set shotReq booleans and the usual sleep/reset sequence
            actions.add(new InstantAction(() -> {
                if (feeder == 1 || feeder == 2) shotReqFR.set(true);  // 1, 2 = front/right feeder
                else shotReqBL.set(true);               // 0 = back/left feeder
            }));
            actions.add(new SleepAction(0.3)); // allow an interval of requesting in case the initial request is overridden
            actions.add(new InstantAction(() -> {
                shotReqFR.set(false);
                shotReqBL.set(false);
            }));
            actions.add(new SleepAction(0.6)); // feed`er completes the state machine
        }
        SequentialAction seq = new SequentialAction(actions);
        return seq;
    }

    public static void display(char[] arr){
        String message = "";
        for (char c : arr) message += c + " ";
        RobotLog.d(message);
    }
    public static void display(int[] arr){
        String message = "";
        for (int j : arr) message += j + " ";
        RobotLog.d(message);
    }
}