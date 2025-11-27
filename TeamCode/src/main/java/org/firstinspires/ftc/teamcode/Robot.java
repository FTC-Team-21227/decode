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


// FULL ROBOT CLASS: drive, flywheel, turret, hood, feeder, intake, camera
public class Robot {
    private static Robot instance = null;

    // --------------SUBSYSTEMS---------------------------------------------------------------------
    AprilDrive drive2;
    Intake intake;
    Feeder feeder;
    Flywheel flywheel;
    Turret turret;
    Hood hood;
    AprilTagLocalization2 camera; // Camera subsystem used in AprilDrive and Obelisk detection
//    Voltage voltageSensor;

    // ---------------BOOLEANS AND ATTRIBUTES-------------------------------------------------------
    boolean shotReqFeederType = true; // True = front feeder
    boolean lockStarted = false; // Lock shooting values to allow manual adjustment
    boolean tapping = false;
    double turretAngle;
    double radps; // Flywheel speed
    double theta; // Hood angle
    double chainShotCount = 1; // How many balls to shoot consecutively
    double lastTime = 0; // Previous time, used to calculate loop time
    int driveSideSign = -1; // Alliance color changes forward vs backwards direction
    Pose2d txWorldPinpoint = new Pose2d(0,0,Math.PI); // Robot pose based on pinpoint
    PoseVelocity2d robotVel = new PoseVelocity2d(new Vector2d(0,0),0);

    // ---------------------------TIMERS------------------------------------------------------------
    ElapsedTime feederTimer;
    ElapsedTime aprilTimer; //gonna use aprilTimer to also track loop times. Sorry, not sorry.
    ElapsedTime intakeTimer; //this is so cancer

    // --------------------------- ROBOT ENUMS------------------------------------------------------
    // Enum that manages shooter state machine
    private enum LaunchState {
        IDLE,
        SPIN_UP_FRONT,
        SPIN_UP_BACK,
        FEED_FRONT,
        FEED_BACK,
        LAUNCHING,
        FEED_DOWN,
    } private LaunchState launchState; // Instance

    // Enum that switches between pinpoint and camera localization
    private enum DriveState {
        RELATIVE,
        ABSOLUTE, // Drive with AprilTag localization for absolute position on field
        ABSOLUTE2
    } private DriveState driveState; // Instance

    // Enum that stores the alliance color, accessible globally
    public enum Color {
        RED,
        BLUE
    } public Color color; // Instance

    // Enum that stores the opmode type, accessible globally
    public enum OpModeState {
        AUTO,
        AUTO_FAR,
        TELEOP,
        TELEOP_FAR
    } public OpModeState opModeState; // Instance

    // -------------------PUBLIC ROBOT CONSTRUCTOR--------------------------------------------------
    /**
     * Creates a robot instance with field-relative position starting at initialPose, and mirrors
     * shooting position and goal positions based on alliance color
     * @param initialPose
     * @param color
     */
    public Robot(Pose2d initialPose, Color color){
        // Poses are mirrored if BLUE
        this.color = color;
        if (this.color==Color.RED) {
            driveSideSign = -1;
            Positions.goalPos = Constants.goalPos;
            // Where the robot will shoot from:
            Positions.autoShotPose = Constants.autoShotPose;
            Positions.autoShotPose_Far = Constants.autoShotPose_Far;
            Positions.teleShotPose = Constants.teleShotPose;
            Positions.deltaH = Constants.deltaH;
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
            RobotLog.d("It's Blue");
        }
        else {
            RobotLog.d("What...");
        }
        // Robot's field relative pose, which starts at initialPose
        txWorldPinpoint = initialPose;
//        voltageSensor = new Voltage(hardwareMap.get(VoltageSensor.class,"Control Hub"));
    }

    // --------------------SINGLETON CONSTRUCTOR AND GETTER-----------------------------------------
    /**
     * Create one instance of robot (singleton).
     * NOTE: ALL DEVICES MUST BE REINITIALIZED BEFORE EVERY OPMODE, THEY ARE NOT SAVED.
     */
    // Get singleton instance
    public static Robot getInstance(Pose2d initialPose, Color color){
        if (instance == null || instance.opModeState == OpModeState.TELEOP){
            RobotLog.d("making a new instance");
            instance = new Robot(initialPose, color);
        }
        else RobotLog.d("keeping the instance");
        return instance;
    }

    // Create new instance
    public static Robot startInstance(Pose2d initialPose, Color color){
        instance = new Robot(initialPose, color);
        return instance;
    }

    // Clear singleton instance
    public static void clearInstance(){
        instance = null;
    }


    // --------------------DRIVETRAIN CLASS OF ROBOT------------------------------------------------
    /**
     * Resets localizer using AprilTags
     * Extends Roadrunner MecanumDrive Class
     */
    public class AprilDrive extends MecanumDrive{
        // -------------------Public AprilDrive Constructor-----------------------------------
        public AprilDrive(HardwareMap hardwareMap, Pose2d initialPose, Color color){
            super(hardwareMap,initialPose,color);
        }

        // --------------------AprilTag Re-localization Method--------------------------------
        @SuppressLint("DefaultLocale")
        public boolean relocalize(Telemetry telemetry) {
            // Update the pinpoint velocity estimate as normal
            robotVel = super.updatePoseEstimate();
            PoseVelocity2d vel = robotVel;

            // ------------------------Exit conditions-----------------
            // Don't relocalize if robot moving too fast (motion blur)
            if (vel.linearVel.norm() > 1.0 || Math.toDegrees(vel.angVel) > 1.0) {
                return false; // EXIT METHOD
            }

            // Get camera's pose (field-relative) using AprilTag
            Pose2d poseWorldTurret = camera.update(telemetry);
            // If no pose was found, default to the pinpoint localizer
            if (poseWorldTurret == null){
                return false; // EXIT METHOD
            }

            // ------------------------Continue------------------------
            // Add 90 degrees to change AprilTag heading information to same orientation as robot.
            poseWorldTurret = new Pose2d(poseWorldTurret.position,poseWorldTurret.heading.toDouble() + Math.PI/2);

            // Transforming the turret pose into the robot (FIELD RELATIVE) pose--> Pose multiplication: pWR = pWT * pRT^-1.
            // The calculation above is now unnecessary because the camera is mounted on the robot, not the turret
            Pose2d poseWorldRobot = poseWorldTurret/*.times(turret.getPoseRobotTurret().inverse())*/;

            // Set the localizer pose to the current field-relative pose based on AprilTag reading.
            localizer.setPose(poseWorldRobot);

            // Telemetry displays robot position and heading information
            telemetry.addLine(String.format("Pose XY %6.1f %6.1f  (inch)",
                    poseWorldRobot.position.x,
                    poseWorldRobot.position.y));
            telemetry.addLine(String.format("Pose Heading %6.1f  (rad)",
                    poseWorldRobot.heading.toDouble()
            ));
            return true; // Returns true for successful relocalization
        }

        // ---------------------------Draw Robot Method---------------------------------------
        public void drawPose(@NonNull TelemetryPacket p){
            p.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(p.fieldOverlay(), localizer.getPose());
            p.fieldOverlay().setStroke("#00FF00");
            Drawing.drawRobot(p.fieldOverlay(),localizer.getPose().times(Constants.turretPos));
        }

        // --------------------P2P Method-----------------------------------------------------
        /**
         * Point-to-point action: drives the robot from the current pose to targetPose using a
         * simple proportional controller (position + heading) and the same wheel kinematics/feedforward
         * approach used in FollowTrajectoryAction/TurnAction.
         *
         * Usage: Actions.run(mecanumDrive.p2pAction(targetPose, 20.0, 1.0, Math.toRadians(3)));
         */
        public void runp2p(@NonNull TelemetryPacket packet) {
            Pose2dDual<Time> targetDual = Pose2dDual.constant(Positions.teleShotPose, 1);  // zero velocity/accel

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
            Pose2d error = Positions.teleShotPose.minusExp(drive2.localizer.getPose()); // Error pose based on difference between current and target pos
            packet.put("xError", error.position.x);
            packet.put("yError", error.position.y);
            packet.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));
            // Stop adjusting if error not too large
            if (error.position.sqrNorm() < 4 && Math.abs(error.heading.toDouble()) < 2){
                drive2.leftFront.setPower(0);
                drive2.leftBack.setPower(0);
                drive2.rightBack.setPower(0);
                drive2.rightFront.setPower(0);
            }
            // Set wheel power
            drive2.leftFront.setPower(leftFrontPower);
            drive2.leftBack.setPower(leftBackPower);
            drive2.rightBack.setPower(rightBackPower);
            drive2.rightFront.setPower(rightFrontPower);
//                Canvas c = packet.fieldOverlay();
//                drawPoseHistory(c);
//                c.setStroke("#FF9800");
//                Drawing.drawRobot(c, targetPose);
//                c.setStroke("#3F51B5");
//                Drawing.drawRobot(c, localizer.getPose());

//                started = true;
        }
    } // END OF APRILDRIVE CLASS

    // ---------------------------AUTON INITIALIZATION----------------------------------------------
    // Initialize and set motors, shooter, timers
    public void initAuto(HardwareMap hardwareMap, Telemetry telemetry, OpModeState opModeState) {
        if (opModeState == OpModeState.AUTO_FAR){
            Positions.goalPos = handleVector(Constants.goalPos_far);
            Positions.deltaH = Constants.deltaH_far;
        }
        intake = new Intake(hardwareMap);
        camera = new AprilTagLocalization2(hardwareMap);
        drive2 = new AprilDrive(hardwareMap, txWorldPinpoint, color);
        feeder = new Feeder(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        turret = new Turret(hardwareMap);
        hood = new Hood(hardwareMap);

        this.opModeState = opModeState;
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
        Positions.flywheelPower = Constants.flywheelPower;
        Positions.flywheelPowerOffset = 0;
        Positions.turretAngleManualOffset = 0;
        Positions.hoodAngleManualOffset = 0;
        Constants.turretAngleOffset = 0;
        Constants.hoodAngleOffset = 0;

        if (!Constants.MINIMIZE_TELEMETRY) {
            RobotLog.dd("goal pos", " " + Positions.goalPos.x + " " + Positions.goalPos.y);
            RobotLog.dd("auto shot pose", " " + Positions.autoShotPose.position.x + " " + Positions.autoShotPose.position.y + " " + Positions.autoShotPose.heading.toDouble());
            RobotLog.dd("auto shot pose far", " " + Positions.autoShotPose_Far.position.x + " " + Positions.autoShotPose_Far.position.y + " " + Positions.autoShotPose_Far.heading.toDouble());
            RobotLog.dd("tele shot pose", " " + Positions.teleShotPose.position.x + " " + Positions.teleShotPose.position.y + " " + Positions.teleShotPose.heading.toDouble());
            RobotLog.dd("initial pose", " " + txWorldPinpoint.position.x + " " + txWorldPinpoint.position.y + " " + txWorldPinpoint.heading.toDouble());
            RobotLog.dd("deltaH", " " + Positions.deltaH);
        }
        telemetry.addData("goal pos", " " + Positions.goalPos.x + " " + Positions.goalPos.y);
        telemetry.addData("auto shot pose far", " " + Positions.autoShotPose_Far.position.x + " " +  Positions.autoShotPose_Far.position.y + " " + Positions.autoShotPose_Far.heading.toDouble());
        telemetry.addData("tele shot pose", " " + Positions.teleShotPose .position.x + " " +  Positions.teleShotPose .position.y + " " + Positions.teleShotPose .heading.toDouble());
        telemetry.addData("initial pose", " " + txWorldPinpoint.position.x + " " + txWorldPinpoint.position.y + " " + txWorldPinpoint.heading.toDouble());
        telemetry.addData("auto shot pose", " " + Positions.autoShotPose.position.x + " " +  Positions.autoShotPose.position.y + " " + Positions.autoShotPose.heading.toDouble());
        telemetry.addData("deltaH"," "+Positions.deltaH);
        telemetry.addData("Status", "Initialized"); //IMPORTANT
//        telemetry.update();
    }

    // -----------------------TELEOP INITIALIZATION-------------------------------------------------
    public void initTeleop(HardwareMap hardwareMap, Telemetry telemetry) {
//        Constants.turretTargetRangeOffset = (Constants.turretLowAngle + Constants.turretHighAngle )/2.0;
        // Initialize subsystems
        intake = new Intake(hardwareMap);
//        camera = new AprilTagLocalization2(hardwareMap);
        drive2 = new AprilDrive(hardwareMap, txWorldPinpoint, color);
        feeder = new Feeder(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        turret = new Turret(hardwareMap);
        hood = new Hood(hardwareMap);

        // Set enums
        opModeState = OpModeState.TELEOP;
        launchState = LaunchState.IDLE;
        driveState = DriveState.RELATIVE;

        // Start timers
        feederTimer = new ElapsedTime();
        aprilTimer = new ElapsedTime();
        intakeTimer = new ElapsedTime();

        // Constants
        Positions.drivePower = Constants.drivePower_Tele;
        Positions.turretClip0 = Constants.turretClip0_tele;
        Positions.turretClip1 = Constants.turretClip1_tele; //CHANGE BACK
        Positions.flywheelPower = Constants.flywheelPower_Tele;
        Positions.flywheelPowerOffset = 0;
        Positions.turretAngleManualOffset = 0;
        Positions.hoodAngleManualOffset = 0;
        Constants.turretAngleOffset = 0;
        Constants.hoodAngleOffset = 0;

        // Logs and telemetry
        if (!Constants.MINIMIZE_TELEMETRY) {
            RobotLog.dd("goal pos", " " + Positions.goalPos.x + " " + Positions.goalPos.y);
            RobotLog.dd("auto shot pose", " "
                    + Positions.autoShotPose.position.x + " "
                    + Positions.autoShotPose.position.y + " "
                    + Positions.autoShotPose.heading.toDouble());
            RobotLog.dd("auto shot pose far", " "
                    + Positions.autoShotPose_Far.position.x + " "
                    + Positions.autoShotPose_Far.position.y + " "
                    + Positions.autoShotPose_Far.heading.toDouble());
            RobotLog.dd("tele shot pose", " "
                    + Positions.teleShotPose.position.x + " "
                    + Positions.teleShotPose.position.y + " "
                    + Positions.teleShotPose.heading.toDouble());
            RobotLog.dd("initial pose", " "
                    + txWorldPinpoint.position.x + " "
                    + txWorldPinpoint.position.y + " "
                    + txWorldPinpoint.heading.toDouble());
            RobotLog.dd("deltaH", " " + Positions.deltaH);
            telemetry.addData("goal pos", " "
                    + Positions.goalPos.x + " "
                    + Positions.goalPos.y);
            telemetry.addData("auto shot pose far", " "
                    + Positions.autoShotPose_Far.position.x + " "
                    + Positions.autoShotPose_Far.position.y + " "
                    + Positions.autoShotPose_Far.heading.toDouble());
            telemetry.addData("tele shot pose", " "
                    + Positions.teleShotPose.position.x + " "
                    + Positions.teleShotPose.position.y + " "
                    + Positions.teleShotPose.heading.toDouble());
            telemetry.addData("initial pose", " "
                    + txWorldPinpoint.position.x + " "
                    + txWorldPinpoint.position.y + " "
                    + txWorldPinpoint.heading.toDouble());
            telemetry.addData("auto shot pose", " "
                    + Positions.autoShotPose.position.x + " "
                    + Positions.autoShotPose.position.y + " "
                    + Positions.autoShotPose.heading.toDouble());
            telemetry.addData("deltaH", " " + Positions.deltaH);
        }
        telemetry.addData("Status", "Initialized"); // IMPORTANT
        if (!Constants.MINIMIZE_TELEMETRY) {
            RobotLog.dd("goal pos", " " + Positions.goalPos.x + " " + Positions.goalPos.y);
            RobotLog.dd("auto shot pose", " " + Positions.autoShotPose.position.x + " " + Positions.autoShotPose.position.y + " " + Positions.autoShotPose.heading.toDouble());
            RobotLog.dd("auto shot pose far", " " + Positions.autoShotPose_Far.position.x + " " + Positions.autoShotPose_Far.position.y + " " + Positions.autoShotPose_Far.heading.toDouble());
            RobotLog.dd("tele shot pose", " " + Positions.teleShotPose.position.x + " " + Positions.teleShotPose.position.y + " " + Positions.teleShotPose.heading.toDouble());
            RobotLog.dd("initial pose", " " + txWorldPinpoint.position.x + " " + txWorldPinpoint.position.y + " " + txWorldPinpoint.heading.toDouble());
            RobotLog.dd("deltaH", " " + Positions.deltaH);
        }
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

    // --------------------WHEN TELEOP STARTS-------------------------------------------------------
    public void startTeleop(){
        feeder.downBL();
        feeder.downFR();
    }


    // ---------------------------ROBOT CONSTANTS CLASS---------------------------------------------
    public static class Constants{
        public final static boolean MINIMIZE_TELEMETRY = false;
        public final static boolean friedFeed = true;
        public static double flywheelPower = 2.32;
        public static double flywheelPower_Tele = 2.32;
        public static double hoodAngleOffset = 0;
        public final static double deltaH = 39; //height of wall = 54, height of ball exit = 18 inch - 3 inch //32; //height of turret = 8.436535433
        public final static double deltaH_far = 41;
        // GOAL POSES
        public final static Vector2d goalPos = new Vector2d(-58.3727-5.3,55.6425+5.3); //distance to triangle center = 7.5 inch
        public final static Vector2d goalPos_far = new Vector2d(-70,55.6425+12); //distance to triangle center = 7.5 inch
        // SHOT POSES
        public final static Pose2d autoShotPose = new Pose2d(-43,33,Math.toRadians(90)); //new Pose2d(-12,15,Math.toRadians(90));
        public final static Pose2d autoShotPose_Far = new Pose2d(56,12,Math.toRadians(120));
        public final static Pose2d teleShotPose = new Pose2d(0,0,goalPos.angleCast().toDouble()+Math.PI);
        public final static Pose2d teleShotPose_Far = new Pose2d(0,0,goalPos.angleCast().toDouble()+Math.PI);
        // FLYWHEEL
        public final static double spinUpTimeout = 2;
        public final static double kP = 0.052, kI = 0, kD = 0.000, kF = 10 /*kF will be removed in our new version*/, kS = 0.65, kV = 0.00450;
        // FEEDER
        public final static double feederPower = 1.0;
        public final static double slowIntakePower = 0.7;
        public final static double intakePower = 1.0;
        public final static double outtakePower = 0.6;
        public final static double intakePulseTime = 0.55;
        public final static double outtakePulseTime = 0.15;
        public final static double outtakePulseTime_Tele = 0.15;
        public final static double intakeStabilizeTime = 0.4;
        public final static double feedTime = 0.5; //0.5; //probably increase to 0.6
        public final static double feedDownTime = 0.4; //less than feedUpTime probably
        // HOOD
        public final static double hoodLowAngle = 65.36742754 * Math.PI/180; // the traj angle from horizonatla (rad) //75 //0;
        public final static double hoodHighAngle = 24.47717215 * Math.PI/180; //30 //50*Math.PI/180; //the traj angle from horizontal 55; // Highest actual degree is 41
        public final static double hoodScale0 = 0.15; //0.27;
        public final static double hoodScale1 = 0.83; //1; //0.85;
        // TURRET
        /**
         * turret 0 = 0.48
         * oldest ones also work
         * for backwards turret, add math.pi here
         */
        public static double turretAngleOffset = 0;
        public final static Pose2d turretPos = new Pose2d(-1.512,-0.12224409,0);
        public final static double turretHighAngle = 3*Math.PI/2; //164.7*Math.PI/180; //220*Math.PI/180;; //355*Math.PI/180; // //140*Math.PI/180; // In rad, pos = 1
        public final static double turretLowAngle = Math.PI/2; //-175*Math.PI/180; //-40*Math.PI/180;; // //-208*Math.PI/180; // In rad (= old -330 deg)
//        public final static double turretTargetRangeOffset = turretHighAngle-Math.PI; //offset from (-pi,pi)
        // Offset from (-pi,pi) to (midpoint-pi, midpoint+pi), i.e. shift midpoint from 0 to new midpoint
        public final static double turretTargetRangeOffset = Math.PI/2; //(turretLowAngle + turretHighAngle )/2.0; //turretHighAngle-Math.PI;
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


    // ---------------------------POSITIONS CLASS---------------------------------------------------
    // These are mobile positions, which are mirrored depending on color. Also includes scalars that are toggled between different values. For better code readability and error proofing.
    public static class Positions{
        public static Vector2d goalPos = Constants.goalPos;
        // Where the robot will shoot from:
        public static Pose2d autoShotPose = Constants.autoShotPose;
        public static Pose2d autoShotPose_Far = Constants.autoShotPose_Far;
        public static Pose2d teleShotPose = Constants.teleShotPose;
        public static double deltaH = Constants.deltaH;
        public static double drivePower = Constants.drivePower;
        public static double turretClip0 = Constants.turretClip0;
        public static double turretClip1 = Constants.turretClip1;
        public static double flywheelPower = Constants.flywheelPower;
        public static double flywheelPowerOffset = 0;
        public static double turretAngleManualOffset = 0;
        public static double hoodAngleManualOffset = 0;
    }


    // ---------------------------TELEOP FIELD-CENTRIC DRIVING--------------------------------------
    public boolean driveFieldCentric(double forward, double right, double rotate, boolean slowMode, boolean p2p) {
        // Normal vs. slow mode
        if (!p2p) {
            if (slowMode) {
                Positions.drivePower = Constants.drivePower_Slow;
            } else {
                Positions.drivePower = Constants.drivePower_Tele;
            }
            double Heading_Angle = drive2.localizer.getPose().heading.toDouble();
            double mag = Math.sqrt(forward * forward + right * right);
            double Motor_FWD_input = right * mag * driveSideSign; //forward * mag;
            double Motor_Side_input = forward * mag * driveSideSign; //-right * mag;
            double Motor_fwd_power = Math.cos(Heading_Angle) * Motor_FWD_input - Math.sin(Heading_Angle) * Motor_Side_input;
            double Motor_side_power = (Math.cos(Heading_Angle) * Motor_Side_input + Math.sin(Heading_Angle) * Motor_FWD_input) * MecanumDrive.PARAMS.inPerTick / MecanumDrive.PARAMS.lateralInPerTick; //*1.5
            double Motor_Rotation_power = rotate * 0.9/* * driveSideSign*/; //0.5
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
        else{
            RobotLog.a("Robotvel: " + robotVel.linearVel.x + ", " + robotVel.linearVel.y + ", " + robotVel.angVel);
//            drive2.runp2p(packet);
            Pose2dDual<Time> targetDual = Pose2dDual.constant(Positions.teleShotPose, 0);  // zero velocity/accel

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    MecanumDrive.PARAMS.axialGain, MecanumDrive.PARAMS.lateralGain, MecanumDrive.PARAMS.headingGain,
                    MecanumDrive.PARAMS.axialVelGain, MecanumDrive.PARAMS.lateralVelGain, MecanumDrive.PARAMS.headingVelGain
            ).compute(targetDual, drive2.localizer.getPose(), robotVel);

            MecanumKinematics.WheelVelocities<Time> wheelVels = drive2.kinematics.inverse(command);
            double voltage = drive2.voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(MecanumDrive.PARAMS.kS,
                    MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick, MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
//                mecanumCommandWriter.write(new MecanumCommandMessage(
//                        voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
//                ));

//            // Telemetry lines
//            // Robot current position
//            packet.put("x", localizer.getPose().position.x);
//            packet.put("y", localizer.getPose().position.y);
//            packet.put("heading (deg)", Math.toDegrees(localizer.getPose().heading.toDouble()));
//            // Position error
            Pose2d error = Positions.teleShotPose.minusExp(drive2.localizer.getPose()); // Error pose based on difference between current and target pos
//            packet.put("xError", error.position.x);
//            packet.put("yError", error.position.y);
//            packet.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));
            // Stop adjusting if error not too large
            if (error.position.sqrNorm() < 4 && Math.abs(error.heading.toDouble()) < 2){
                drive2.leftFront.setPower(0);
                drive2.leftBack.setPower(0);
                drive2.rightBack.setPower(0);
                drive2.rightFront.setPower(0);
                return false;
            }
            // Set wheel power
            drive2.leftFront.setPower(leftFrontPower);
            drive2.leftBack.setPower(leftBackPower);
            drive2.rightBack.setPower(rightBackPower);
            drive2.rightFront.setPower(rightFrontPower);
//                Canvas c = packet.fieldOverlay();
//                drawPoseHistory(c);
//                c.setStroke("#FF9800");
//                Drawing.drawRobot(c, targetPose);
//                c.setStroke("#3F51B5");
//                Drawing.drawRobot(c, localizer.getPose());

//                started = true;
            return true;
        }
        return false;
    }
    public boolean driveFieldCentric_Smoth(double forward, double right, double rotate, boolean slowMode, boolean p2p) {
        if (!p2p) {
            if (slowMode) {
                Positions.drivePower = Constants.drivePower_Slow;
            } else {
                Positions.drivePower = Constants.drivePower_Tele;
            }
            double Heading_Angle = drive2.localizer.getPose().heading.toDouble();
            double mag = Math.sqrt(forward * forward + right * right);
            double Motor_FWD_input = right * mag * driveSideSign; //forward * mag;
            double Motor_Side_input = forward * mag * driveSideSign; //-right * mag;
            double Motor_fwd_power = Math.cos(Heading_Angle) * Motor_FWD_input - Math.sin(Heading_Angle) * Motor_Side_input;
            double Motor_side_power = (Math.cos(Heading_Angle) * Motor_Side_input + Math.sin(Heading_Angle) * Motor_FWD_input) * MecanumDrive.PARAMS.inPerTick / MecanumDrive.PARAMS.lateralInPerTick; //*1.5
            double Motor_Rotation_power = rotate * 0.9/* * driveSideSign*/; //0.05
            drive2.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            Motor_fwd_power,
                            Motor_side_power
                    ),
                    Motor_Rotation_power
            ));
        }
        else{
            RobotLog.a("Robotvel: " + robotVel.linearVel.x + ", " + robotVel.linearVel.y + ", " + robotVel.angVel);
//            drive2.runp2p(packet);
            Pose2dDual<Time> targetDual = Pose2dDual.constant(Positions.teleShotPose, 0);  // zero velocity/accel

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    MecanumDrive.PARAMS.axialGain, MecanumDrive.PARAMS.lateralGain, MecanumDrive.PARAMS.headingGain,
                    MecanumDrive.PARAMS.axialVelGain, MecanumDrive.PARAMS.lateralVelGain, MecanumDrive.PARAMS.headingVelGain
            ).compute(targetDual, drive2.localizer.getPose(), robotVel);

            MecanumKinematics.WheelVelocities<Time> wheelVels = drive2.kinematics.inverse(command);
            double voltage = drive2.voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(MecanumDrive.PARAMS.kS,
                    MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick, MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
//                mecanumCommandWriter.write(new MecanumCommandMessage(
//                        voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
//                ));

//            // Telemetry lines
//            // Robot current position
//            packet.put("x", localizer.getPose().position.x);
//            packet.put("y", localizer.getPose().position.y);
//            packet.put("heading (deg)", Math.toDegrees(localizer.getPose().heading.toDouble()));
//            // Position error
            Pose2d error = Positions.teleShotPose.minusExp(drive2.localizer.getPose()); // Error pose based on difference between current and target pos
//            packet.put("xError", error.position.x);
//            packet.put("yError", error.position.y);
//            packet.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));
            // Stop adjusting if error not too large
            if (error.position.sqrNorm() < 4 && Math.abs(error.heading.toDouble()) < 2){
                drive2.leftFront.setPower(0);
                drive2.leftBack.setPower(0);
                drive2.rightBack.setPower(0);
                drive2.rightFront.setPower(0);
                return false;
            }
            // Set wheel power
            drive2.leftFront.setPower(leftFrontPower);
            drive2.leftBack.setPower(leftBackPower);
            drive2.rightBack.setPower(rightBackPower);
            drive2.rightFront.setPower(rightFrontPower);
//                Canvas c = packet.fieldOverlay();
//                drawPoseHistory(c);
//                c.setStroke("#FF9800");
//                Drawing.drawRobot(c, targetPose);
//                c.setStroke("#3F51B5");
//                Drawing.drawRobot(c, localizer.getPose());

//                started = true;
            return true;
        }
        return false;
    }


    // ---------------------------SHOOTER METHOD----------------------------------------------------
    // Calculates and sets hood angle and flywheel RPM. Includes shooter state manager.
    public void updateShooter(boolean shotRequestedFront, boolean shotRequestedBack, boolean shotReqAlt,
                              Telemetry telemetry, boolean setPose, Pose2d setRobotPose,
                              double flywheelChange, boolean hoodUp, boolean hoodDown,
                              boolean turretLeft, boolean turretRight, boolean humanFeed,
                              boolean flywheelOff, boolean frontUp, boolean frontDown,
                              boolean backUp, boolean backDown, boolean moveShot) {
        // Robot pose and velocity
        Pose2d poseRobot = drive2.localizer.getPose(); // Robot pose

        Vector2d linVel = robotVel.linearVel;   // in in/s (consistent with other code)
        double angVel = robotVel.angVel;         // rad/s

        // Shooting trajectory values
        double p = 0.65; // Fraction of time along trajectory from ground to ground
        double g = 386.22; // Gravity (in/s^2)
        double deltaH = Positions.deltaH; // Height difference from shooter to goal
        double flightTime = Math.sqrt(2 * deltaH / (p * g * (1 - p))); // Ball trajectory time from ground to ground

        // Moving shot correction
        Pose2d correctedPoseRobot = new Pose2d(poseRobot.position.plus(linVel.times(flightTime)),poseRobot.heading.plus(angVel*flightTime)); //angVel thing is incorrect, the heading should be same as original but the turning should offset the virtual pose a tiny bit more.
        if (moveShot){
            poseRobot = correctedPoseRobot;
        }

        // Locking shooter values
        // if setPose just became true, recompute next loop:
        // If user wants to lock shooter values, set setPose to true, and give it a Pose setRobotPose
        if (!setPose) lockStarted = false;
        if (setPose && setRobotPose != null){
            poseRobot = setRobotPose; // Calculate shooting values on where we plan to shoot
//            RobotLog.d("Correct auto pose!");
        }
        else {
            if (!Constants.MINIMIZE_TELEMETRY) {
                if (opModeState == OpModeState.AUTO)
                    RobotLog.d("This is auto right now, it's bad because it's calculating with the wrong pose");
            }
        }

        // Pose with the TURRET's position on the robot and ROBOT's heading, CALCULATES VECTOR
        Pose2d pose = poseRobot.times(Constants.turretPos);
        Vector2d goalVector = Positions.goalPos.minus(pose.position);

        double distance = goalVector.norm(); // Horizontal distance
        double goalVectorAngle = goalVector.angleCast().toDouble();

        if (!setPose || !lockStarted) {
            if (setPose) lockStarted = true;
            try {
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
                if (!Constants.MINIMIZE_TELEMETRY) {
                    RobotLog.dd("SHOOTER CALC FAILED MATH", e.getMessage());
                    lockStarted = false;
                }
            }
        }

        if (opModeState == OpModeState.TELEOP_FAR && color == Color.RED) {
            if (shotReqFeederType) {//front
                Positions.flywheelPower = 2.481; //change these for teleop
                Constants.turretAngleOffset = -4 * Math.PI / 180;
            }
            else{
                Positions.flywheelPower = 2.305;
                Constants.turretAngleOffset = 3 * Math.PI / 180;
            }
        }
        else if (opModeState == OpModeState.TELEOP_FAR && color == Color.BLUE) {
            if (shotReqFeederType) {//front
                Positions.flywheelPower = 2.145; //change these for teleop
                Constants.turretAngleOffset = -2 * Math.PI / 180;
            }
            else{
                Positions.flywheelPower = 2.239;
                Constants.turretAngleOffset = -4 * Math.PI / 180;
            }
        }
        // Manually control hood
        double hoodChange = 0;
        if (hoodUp) hoodChange -= Math.PI/180;
        if (hoodDown) hoodChange += Math.PI/180;
        Positions.hoodAngleManualOffset += hoodChange;
        // Manually control turret
        double turretChange = 0;
        if (turretLeft) turretChange -= Math.PI/180;
        if (turretRight) turretChange += Math.PI/180;
        Positions.turretAngleManualOffset += turretChange;

        // Adjust flywheel RPM
        double delta = 0;
        // If the flywheel power change is a big number, adjust little by little
        if (Math.abs(flywheelChange) > 0.95) delta = Math.signum(flywheelChange)*0.001;
        Positions.flywheelPowerOffset += delta;

        // Spin reverse direction for human feeding (when button is held down)
        if (humanFeed){
            //maybe also keep hood low and turret at constant pos
            radps = -800 / 28.0 * Math.PI * 2;
            theta = Math.PI/2;
            turretAngle = 0;
        }
        if (flywheelOff){ //not actually flywheelOff but really changing the disable status
            flywheel.FLYWHEEL.getMotor().setPower(0);
            telemetry.addLine("trying to 0");
        }
        else {
            flywheel.spinTo(radps * 28 / Math.PI / 2 * (Positions.flywheelPower + Positions.flywheelPowerOffset));
        }
        // Set hood angle to theta (convert to servo position)
        hood.turnToAngle(theta+Constants.hoodAngleOffset+Positions.hoodAngleManualOffset);
        turret.turnToRobotAngle(turretAngle+Constants.turretAngleOffset+Positions.turretAngleManualOffset);

        //use +-1 of requesting state

        if (frontUp) feeder.upFR();
        else if (frontDown) feeder.downFR();
        else if (backUp) feeder.upBL();
        else if (backDown) feeder.downBL();
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
//                            shotReqFeederType = !shotReqFeederType;
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
//                        shotReqFeederType = !shotReqFeederType;
                        feederTimer.reset();
                    }
                }
                // Normal shooting
                else if (!shotReqAlt) {
                    chainShotCount = 1;
                    shotReqFeederType = false;
                }
                if (shotRequestedFront/* && feederTimer.seconds()>Constants.feedTime*/) {// After feeding is done. change req state here too
                        launchState = LaunchState.SPIN_UP_FRONT;
//                        shotReqFeederType = false; //next RB will be back. These dont matter anymore
                        feederTimer.reset();
                }
                else if (shotRequestedBack/* && feederTimer.seconds()>Constants.feedTime*/) {
                        launchState = LaunchState.SPIN_UP_BACK;
//                        shotReqFeederType = true;
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
                    shotReqFeederType = !shotReqFeederType;
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
        telemetry.addData("flywheel power scale factor", Positions.flywheelPower + Positions.flywheelPowerOffset);
        telemetry.addData("State", launchState);
        telemetry.addData("Next feeder type", shotReqFeederType);
        if (!Constants.MINIMIZE_TELEMETRY) {
            telemetry.addData("human feed", humanFeed); //NOT IMPORTANT
            telemetry.addData("setPose", setPose); //NOT IMPORTANT
        }
        telemetry.addLine("robotTurretPose (inchxinchxdeg): " + pose.position.x+" "+pose.position.y+" "+pose.heading.toDouble()); //NOT IMPORTANT
        telemetry.addLine("goalVector (inchxinch): " + goalVector.x+" "+goalVector.y);
        telemetry.addLine("goalPos (inchxinch): " + Positions.goalPos.x+" "+Positions.goalPos.y); //NOT IMPORTANT
        if (!Constants.MINIMIZE_TELEMETRY) {
            telemetry.addData("goalVector angle (rad to deg)", Math.toDegrees(goalVectorAngle)); //NOT IMPORTANT
            telemetry.addData("distance to goal (inch)", distance); //NOT IMPORTANT
        }
        telemetry.addData("turret angle (rad to deg)", turretAngle*180/Math.PI);
        telemetry.addData("turret angle offset (rad to deg)", Constants.turretAngleOffset*180/Math.PI);
        telemetry.addData("turret angle offset manual(rad to deg)", Positions.turretAngleManualOffset*180/Math.PI);
        if (!Constants.MINIMIZE_TELEMETRY) {
            telemetry.addData("turret get angle (rad to deg)", turret.getTurretRobotAngle() * 180 / Math.PI); //NOT IMPORTANT
        }
        telemetry.addData("turret pos", turret.turret.getPosition()); //NOT IMPORTANT
        telemetry.addData("hood theta (rad to deg)", theta*180/Math.PI);
        telemetry.addData("hood angle offset (rad to deg)", Constants.hoodAngleOffset*180/Math.PI);
        telemetry.addData("hood angle offset manual (rad to deg)", Positions.hoodAngleManualOffset*180/Math.PI);
        if (!Constants.MINIMIZE_TELEMETRY) {
            telemetry.addData("hood get angle (rad to deg)", hood.getAngle()); //NOT IMPORTANT
        }
        telemetry.addData("hood pos", hood.HOOD.getPosition()); //NOT IMPORTANT
        telemetry.addData("targetVel (rad/s to tick/s)", radps*28/Math.PI/2*(Positions.flywheelPower+Positions.flywheelPowerOffset));
        if (!Constants.MINIMIZE_TELEMETRY) {
            telemetry.addData("targetVel (rad/s)", radps); //NOT IMPORTANT
        }
        telemetry.addData("motorSpeed (tick/s)", flywheel.getVel());
        if (!Constants.MINIMIZE_TELEMETRY) {
            telemetry.addData("motorSpeed (tick/s to rad/s)", flywheel.getVel() * 2 * Math.PI / 28); //NOT IMPORTANT
        }
        telemetry.addData("feeder fr pos", feeder.FR_FEEDER.getPosition());
        telemetry.addData("feeder bl pos", feeder.BL_FEEDER.getPosition());
        telemetry.addData("feeder seconds", feederTimer.seconds()); //NOT IMPORTANT
//        telemetry.update();
    }

    /**
     * Returns field-relative robot pose (calculated using turret pose), or returns Pinpoint-recorded
     * pose if no AprilTag detections. Also displays pose information on telemetry.
     */
    public void updateLocalizer(boolean relocalize, boolean relocalize2, Telemetry telemetry){
        switch (driveState){
            case RELATIVE:
                robotVel = drive2.updatePoseEstimate();
//                if (aprilTimer.seconds() > 10){
//                    driveState = DriveState.ABSOLUTE;
//                }
                if (relocalize) driveState = DriveState.ABSOLUTE;
                if (relocalize2) driveState = DriveState.ABSOLUTE2;
                break;
            case ABSOLUTE:
                boolean success = drive2.relocalize(telemetry);
                if (success) aprilTimer.reset();
                driveState = DriveState.RELATIVE;
                break;
            case ABSOLUTE2:
                Pose2d newPose = handlePose(new Pose2d(-51.84,51.2636,Math.toRadians(-54.2651)));
                drive2.localizer.setPose(newPose);
                txWorldPinpoint = newPose;
                Constants.hoodAngleOffset = 0;
                Constants.turretAngleOffset = 0;
                Positions.turretAngleManualOffset = 0;
                Positions.hoodAngleManualOffset = 0;
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
            Positions.teleShotPose = handlePose(Constants.teleShotPose);
            opModeState = OpModeState.TELEOP;
        }
        else { //chagne to be better at far
            Positions.goalPos = handleVector(Constants.goalPos_far);
            Positions.deltaH = Constants.deltaH_far;
            Positions.teleShotPose = handlePose(Constants.teleShotPose_Far);
            opModeState = OpModeState.TELEOP_FAR;
        }
        return close;
    }
//    public double updateVoltage(Telemetry telemetry){
//        double voltage = voltageSensor.updateVoltage();
//        telemetry.addData("volts",voltage);
//        return voltage;
//    }

    public void controlIntake(boolean in, boolean out, boolean stop, boolean tapOut, boolean inSlow, boolean outFast){
//        intake.updateComp();
        if (intakeTimer.seconds()>Constants.outtakePulseTime_Tele){
            if (tapping) {intake.proceed(); tapping = false;}
            if (out) intake.outtake();
            else if (in) intake.intake();
            else if (inSlow) intake.slowIntake();
            else if (stop) intake.stop();
            else if (outFast) intake.strongOuttake();
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
                                       char[] queue, int obeliskID, int shot, OpModeState opModeState, Color color) {

        char[] desired = getDesiredPattern(obeliskID);
        Order = computeFireOrder(queue, desired);
        if (Constants.friedFeed) Order = new int[]{1,2,0};
        if (!Constants.MINIMIZE_TELEMETRY) {
            RobotLog.a("Shohtott " + shot);
            RobotLog.d("Obelisk ID: " + obeliskID);
            RobotLog.d("balls in robot: ");
            display(queue);
            RobotLog.d("balls score order: ");
            display(desired);
            RobotLog.d("feeders move order: ");
            display(Order);
        }
        ArrayList<Action> actions = new ArrayList<>();
        if (opModeState == OpModeState.AUTO_FAR && color == Color.RED) {
            if (Order[0] == 0) {
                actions.add(new InstantAction(() -> Positions.flywheelPower = 2.15));
                actions.add(new InstantAction(() -> Constants.turretAngleOffset = -1 * Math.PI / 180));
            } else if (Order[0] == 1 || Order[0] == 2) {
                actions.add(new InstantAction(() -> Positions.flywheelPower = 2.338));
                actions.add(new InstantAction(() -> Constants.turretAngleOffset = 5 * Math.PI / 180));
            }
        }
        else if (opModeState == OpModeState.AUTO_FAR && color == Color.BLUE) {
//            RobotLog.a("change first");
            if (Order[0] == 0) {
                actions.add(new InstantAction(() -> Positions.flywheelPower = 2.25));
                actions.add(new InstantAction(() -> Constants.turretAngleOffset = 6 * Math.PI / 180));
            } else if (Order[0] == 1 || Order[0] == 2) {
                actions.add(new InstantAction(() -> Positions.flywheelPower = 2.414));
                actions.add(new InstantAction(() -> Constants.turretAngleOffset = 8 * Math.PI / 180));
            }
        }
        if (opModeState == OpModeState.AUTO || opModeState == OpModeState.AUTO_FAR) {
            actions.add(new SleepAction(0.4));
        }
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
            if (i == 2 && (opModeState == OpModeState.AUTO || opModeState == OpModeState.AUTO_FAR)){
                actions.add(new SleepAction(0.4));
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
            if (opModeState == OpModeState.AUTO_FAR && color == Color.RED && i < 2) {
                if (Order[i + 1] == 1 || Order[i + 1] == 2) {
                    actions.add(new SleepAction(0.2));
                    actions.add(new InstantAction(() -> Positions.flywheelPower = 2.338));
                    actions.add(new InstantAction(() -> Constants.turretAngleOffset = 5 * Math.PI / 180));
                } else if (Order[i + 1] == 0) {
                    actions.add(new SleepAction(0.2));
                    actions.add(new InstantAction(() -> Positions.flywheelPower = 2.15));
                    actions.add(new InstantAction(() -> Constants.turretAngleOffset = -1 * Math.PI / 180));
                }
            }
            else if (opModeState == OpModeState.AUTO_FAR && color == Color.BLUE && i < 2) {
//                RobotLog.a("change " + i);
                if (Order[i + 1] == 0) {
                    actions.add(new SleepAction(0.2));
                    actions.add(new InstantAction(() -> Positions.flywheelPower = 2.25));
                    actions.add(new InstantAction(() -> Constants.turretAngleOffset = 6 * Math.PI / 180));
                } else if (Order[i + 1] == 1 || Order[i+1] == 2) {
                    actions.add(new SleepAction(0.2));
                    actions.add(new InstantAction(() -> Positions.flywheelPower = 2.414));
                    actions.add(new InstantAction(() -> Constants.turretAngleOffset = 8 * Math.PI / 180));
                }
            }
            if (i != 2) actions.add(new SleepAction(0.6)); // feed`er completes the state machine
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