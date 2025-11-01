package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.DualNum;
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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;
// Full Robot: drive, flywheel, turret, hood, feeder, intake, camera
// Camera mounted on turret


@Config
public class Robot {
    private static Robot instance = null;
    // Red/blue pose mirroring
    // Lookup table functionality
    // Turret position mapped correctly to robot pose etc.
    // Telemetry
    // Some way to carry the information over to teleop, not have to reinitialize (singleton later)
    AprilDrive drive2; // Drive base of the robot (motors, odometry, pinpoint, and camera - capable of hybrid localization)
    Intake intake; // Motor subsystem that runs continuously throughout the match to spin the intake
    Feeder feeder; // Servo subsystem that runs occasionally to move the ball up the elevator into the flywheel
    Flywheel flywheel; // Motor subsystem that spins to certain target RPM throughout the match
    Turret turret; // Servo subsystem that turns to any robot-relative angle
    Hood hood; // Servo subsystem that raises or lowers the hood anywhere from low to high degrees
    AprilTagLocalization2 camera; // Camera subsystem that is used in AprilDrive and Obelisk detection
    boolean shotReqFeederType = true; //true = front
    boolean lockStarted = false;
    double turretAngle;
    double radps;
    double theta;
    double chainShotCount = 1;

    // Enum that stores the alliance color, accessible globally
    public enum Color {
        RED,
        BLUE
    }
    public final Color color; // Instance of the enum to initialize later

    // Initialize subsystems
    public Robot(HardwareMap hardwareMap, Pose2d initialPose, Color color){
        this.color = color; // Pose mirroring can occur depending on color
        switch (color){
            case RED:
                Constants.goalPos = new Vector2d(-58.3727,55.6425);
                break;
            case BLUE:
                Constants.goalPos = new Vector2d(-58.3727,-55.6425);
                Constants.autoShotPose = mirrorPose(Constants.autoShotPose);
                initialPose = mirrorPose(initialPose);
                break;
        }
        intake = new Intake(hardwareMap);
        camera = new AprilTagLocalization2(hardwareMap);
        drive2 = new AprilDrive(hardwareMap, initialPose);
        feeder = new Feeder(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        turret = new Turret(hardwareMap);
        hood = new Hood(hardwareMap);
    }

    // Create one instance of robot (singleton)
    public static Robot getInstance(HardwareMap hardwareMap, Pose2d initialPose, Color color){
        if (instance == null){
            instance = new Robot(hardwareMap, initialPose, color);
        }
        return instance;
    }
    public void clearInstance(){
        instance = null;
    }
    public class AprilDrive extends MecanumDrive{
        // Reset localizer functions using AprilTags
        public AprilDrive(HardwareMap hardwareMap, Pose2d initialPose){ // Using the parent constructor since we only are creating one method
            super(hardwareMap,initialPose);
        }

        // Every ~10 sec, use the goal AprilTag to re-define the robot's pose relative to the field.
        // NOTE: Maybe want to return a boolean of successful relocalization instead of the pose, which is accessible by other means (keep trying to relocalize until we get a success)
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
            Pose2d poseWorldRobot = poseWorldTurret.times(turret.getPoseRobotTurret().inverse());
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
    }
    private LaunchState launchState;

    private enum DriveState {
        RELATIVE,
        ABSOLUTE // Drive with AprilTag localization for absolute position on field
    }
    private DriveState driveState;

    ElapsedTime feederTimer;
    ElapsedTime aprilTimer;


    // Initialize and set motors, shooter, timers
    public void initAuto(Telemetry telemetry) {
        launchState = LaunchState.IDLE;
        driveState = DriveState.RELATIVE;

        feederTimer = new ElapsedTime();
        aprilTimer = new ElapsedTime();

        Constants.drivePower = 1;

        telemetry.addData("Status", "Initialized");
//        telemetry.update();
    }
    // Initialize and set motors, shooter, timers
    public void initTeleop(Telemetry telemetry) {
        launchState = LaunchState.IDLE;
        driveState = DriveState.RELATIVE;

        feederTimer = new ElapsedTime();
        aprilTimer = new ElapsedTime();

        Constants.drivePower = 0.5;

        telemetry.addData("Status", "Initialized");
//        telemetry.update();
    }

    // Constants
    @Config
    public static class Constants{
        public final static Pose2d turretPos = new Pose2d(1.5,0,0); //1.5
        public static double flywheelPower = 2.8;
        public final static double deltaH = 30;
        public static Vector2d goalPos = new Vector2d(-58.3727,55.6425);
        // Where the robot will shoot from:
        public static Pose2d autoShotPose = new Pose2d(-12,15,Math.toRadians(90));
        public final static Pose2d poseTurretCamera = new Pose2d(0, 3, 0);
        public static double p = 300, i = 0, d = 0, f = 10;

        public final static double feederPower = 1.0;
        public final static double intakePower = 1.0;
        public final static double outtakePower = -1.0;
        public final static double feedTime = 0.5;
        public final static double spinUpTimeout = 2;
        public final static double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
        public final static double FULL_SPEED = 1.0;
        public final static double LAUNCHER_TARGET_VELOCITY = 1125;
        public final static double LAUNCHER_MIN_VELOCITY = 1075;
        // HOOD CONSTANTS
        public final static double hoodLowAngle = 60*Math.PI/180; // the traj angle from horizonatla (rad) //0;
        public final static double hoodHighAngle = 30 * Math.PI/180; //50*Math.PI/180; //the traj angle from horizontal 55; // Highest actual degree is 41
        public final static double hoodScale0 = 0.15; //0.27;
        public final static double hoodScale1 = 0.83; //1; //0.85;
        // TURRET CONSTANTS
        //turret 0 = 0.48
        public final static double turretHighAngle = 220*Math.PI/180; //140*Math.PI/180; // In rad, pos = 1
        public final static double turretLowAngle = -175*Math.PI/180; //-208*Math.PI/180; // In rad (= old -330 deg)
        public final static double turretTargetRangeOffset = turretHighAngle-Math.PI; //offset from (-pi,pi)
        public final static double turretScale0 = 0; //0.11;
        public final static double turretScale1 = 1;
        public final static double feederScale0 = 0;
        public final static double feederScale1 = 1;
        public static double drivePower = 1.0;
    }

    // Drives the robot field-centric
    public void driveFieldCentric(double forward, double right, double rotate) {
        double Heading_Angle = drive2.localizer.getPose().heading.toDouble();
        double mag = Math.sqrt(forward*forward + right*right);
        double Motor_FWD_input = forward * mag;
        double Motor_Side_input = -right * mag;
        double Motor_fwd_power = Math.cos(Heading_Angle) * Motor_FWD_input - Math.sin(Heading_Angle) * Motor_Side_input;
        double Motor_side_power = (Math.cos(Heading_Angle) * Motor_Side_input + Math.sin(Heading_Angle) * Motor_FWD_input) * MecanumDrive.PARAMS.inPerTick / MecanumDrive.PARAMS.lateralInPerTick; //*1.5
        double Motor_Rotation_power = rotate * 0.7; //0.5
        double Motor_power_BL = -(((Motor_fwd_power - Motor_side_power) - Motor_Rotation_power) * Constants.drivePower);
        double Motor_power_BR = -(((Motor_fwd_power + Motor_side_power) + Motor_Rotation_power) * Constants.drivePower);
        double Motor_power_FL = -(((Motor_fwd_power + Motor_side_power) - Motor_Rotation_power) * Constants.drivePower);
        double Motor_power_FR = -(((Motor_fwd_power - Motor_side_power) + Motor_Rotation_power) * Constants.drivePower);
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
    public void updateShooter(boolean shotRequestedFront, boolean shotRequestedBack, boolean shotReqAlt, Telemetry telemetry, boolean setPose, Pose2d setRobotPose, double flywheelChange, boolean hoodUp, boolean hoodDown, boolean turretLeft, boolean turretRight, boolean humanFeed) { // TODO: add some params where u can hardset shooter outputs
        // TODO: replace these with LUT values
        // Assume we have: Vector2d goalPosition
        Pose2d poseRobot = drive2.localizer.getPose();
//        Pose2d pose = new Pose2d(poseRobot.position.plus(Constants.turretPos), poseRobot.heading); // Pose with the TURRET's position and ROBOT's heading
        Pose2d pose = poseRobot.times(Constants.turretPos); // Pose with the TURRET's position and ROBOT's heading
        // if setPose just became true, recompute next loop:
        if (!setPose) lockStarted = false;
        if (setPose){
            pose = setRobotPose;
        }
        Vector2d goalVector = Constants.goalPos.minus(pose.position);


        double p = 0.75; // Fraction of time along trajectory from ground to ground
        double g = 386.22; // Gravity (in/s^2)

        double deltaH = Constants.deltaH; // Height difference from shooter to goal
        double distance = goalVector.norm(); // Horizontal distance
        double goalVectorAngle = goalVector.angleCast().toDouble();

        if (!setPose || !lockStarted) {
            lockStarted = true;
            try {
                double flightTime = Math.sqrt(2 * deltaH / (p * g * (1 - p))); // Ball trajectory time from ground to ground
                theta = Math.atan(deltaH / (distance * (1 - p))); // Ball launch angle of elevation
                double vel = distance / (p * flightTime * Math.cos(theta)); // Ball launch speed
                //        double absoluteAngleToGoal = /*Math.PI + */Constants.goalPos.minus(pose.position).angleCast().toDouble();
                double heading = pose.heading.toDouble();
                turretAngle = goalVectorAngle - heading; // Angle to turn turret to (relative to robot's heading)
                double wheelRadius = 1.89; // Inches, for example
            /*
            double wheelCircumference = Math.PI * wheelDiameter;
            double change = 0;
            if (up) change += 0.001;
            if (down) change -= 0.001;
            StarterRobot.Constants.flywheelPower += change;
             */
                // Convert vel (speed) to rad/s (example calibration: vel = wheelRadius * rad/s
                radps = vel / wheelRadius * Constants.flywheelPower; // RPM
            }
            catch(ArithmeticException e){
                RobotLog.dd("SHOOTER CALC FAILED MATH", e.getMessage());
            }
        }
        else{
            double hoodChange = 0;
            if (hoodUp) hoodChange += 1;
            if (hoodDown) hoodChange -= 1;
            theta += hoodChange;
            double turretChange = 0;
            if (turretLeft) turretChange -= 1;
            if (turretRight) turretChange += 1;
            turretAngle += turretChange;
        }
        // Set flywheel RPM
        // TODO: UNCOMMENT LATER!
        double delta = 0;
        if (Math.abs(flywheelChange) > 0.95) delta = Math.signum(flywheelChange)*0.001;
        Constants.flywheelPower += delta;
        if (humanFeed){
            //maybe also keep hood low and turret at constant pos
            radps = -800 / 28.0 * Math.PI * 2;
            theta = Math.PI/2;
            turretAngle = 0;
        }
        flywheel.spinTo(radps * 28 / Math.PI / 2);
        // Set hood angle to theta (convert to servo position)
        hood.turnToAngle(theta);
        turret.turnToRobotAngle(turretAngle);

        //use +-1 of requesting state

        switch (launchState) {
            case IDLE:
                if (shotReqAlt && feederTimer.seconds()>Constants.feedTime){
                    if (chainShotCount == 2){
                        if (feederTimer.seconds() > Constants.feedTime + 0.6) {
                            intake.stop();
                            if (shotReqFeederType) launchState = LaunchState.SPIN_UP_FRONT;
                            else launchState = LaunchState.SPIN_UP_BACK;
                            shotReqFeederType = !shotReqFeederType;
                            feederTimer.reset();
                        }
                        else if (feederTimer.seconds() > Constants.feedTime + 0.1)
                            intake.intake();
                    }
                    else {
                        if (shotReqFeederType) launchState = LaunchState.SPIN_UP_FRONT;
                        else launchState = LaunchState.SPIN_UP_BACK;
                        shotReqFeederType = !shotReqFeederType;
                        feederTimer.reset();
                    }
                }
                else chainShotCount = 1;
                if (shotRequestedFront && feederTimer.seconds()>Constants.feedTime) {// After feeding is done. change req state here too
                        launchState = LaunchState.SPIN_UP_FRONT;
                        shotReqFeederType = false; //next RB will be back
                        feederTimer.reset();
                }
                else if (shotRequestedBack && feederTimer.seconds()>Constants.feedTime) {
                        launchState = LaunchState.SPIN_UP_BACK;
                        shotReqFeederType = true;
                        feederTimer.reset();
                }
                break;
            case SPIN_UP_FRONT: // SPEED UP FLYWHEEL
                if (flywheel.getVel() > radps * 28 / Math.PI / 2 - 50 || feederTimer.seconds() > Constants.spinUpTimeout) {
                    launchState = LaunchState.FEED_FRONT;
                }
//                launchState = launchState.FEED_FRONT;
                break;
            case SPIN_UP_BACK: // SPEED UP FLYWHEEL
                if (flywheel.getVel() > radps * 28 / Math.PI / 2 - 50 || feederTimer.seconds() > Constants.spinUpTimeout) {
                    launchState = LaunchState.FEED_BACK;
                }
//                launchState = launchState.FEED_BACK;
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
                    launchState = LaunchState.IDLE;
                    feeder.downFR();
                    feeder.downBL();
                    feederTimer.reset();
                    intake.proceed();
                    chainShotCount++;
                }
                break;
        }

        // TELEMETRY LINES
        if (hood.commandedOutsideRange()) telemetry.addLine("WARNING: hood commanded out of its range! Auto set to 0 or 1.");
        if (turret.commandedOutsideRange()) telemetry.addLine("WARNING: turret commanded out of its range! Auto set to 0 or 1.");
        telemetry.addData("flywheel power scale factor", Constants.flywheelPower);
        telemetry.addData("State", launchState);
        telemetry.addData("Next feeder type", shotReqFeederType);
        telemetry.addData("human feed", humanFeed);
        telemetry.addData("setPose", setPose);
        telemetry.addLine("goalVector (inchxinch): " + goalVector.x+" "+goalVector.y);
        telemetry.addLine("goalPos (inchxinch): " + Constants.goalPos.x+" "+Constants.goalPos.y);
        telemetry.addData("goalVector angle (rad to deg)", Math.toDegrees(goalVectorAngle));
        telemetry.addData("distance to goal (inch)", distance);
        telemetry.addData("turret angle (rad to deg)", turretAngle*180/Math.PI);
        telemetry.addData("turret get angle (rad to deg)", turret.getTurretRobotAngle()*180/Math.PI);
        telemetry.addData("turret pos", turret.turret.getPosition());
        telemetry.addData("hood theta (rad to deg)", theta*180/Math.PI);
        telemetry.addData("hood get angle (rad to deg)", hood.getAngle());
        telemetry.addData("hood pos", hood.HOOD.getPosition());
        telemetry.addData("targetVel (rad/s to tick/s)", radps*28/Math.PI/2);
        telemetry.addData("targetVel (rad/s)", radps);
        telemetry.addData("motorSpeed (tick/s)", flywheel.getVel());
        telemetry.addData("motorSpeed (tick/s to rad/s)", flywheel.getVel()*2*Math.PI/28);
        telemetry.addData("feeder fr pos", feeder.FR_FEEDER.getPosition());
        telemetry.addData("feeder bl pos", feeder.BL_FEEDER.getPosition());
        telemetry.addData("feeder seconds", feederTimer.seconds());
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
//                aprilTimer.reset();
                driveState = DriveState.RELATIVE;
                break;
        }
        telemetry.addData("x", drive2.localizer.getPose().position.x);
        telemetry.addData("y", drive2.localizer.getPose().position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive2.localizer.getPose().heading.toDouble()));
    }

    public void controlIntake(boolean in, boolean out, boolean stop){
        if (out) intake.outtake();
        else if (in) intake.intake();
        else if (stop) intake.stop();
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

        // --- Rule 2: If 2 is followed by 1, change 1 → 0 ---
        for (int i = 0; i < order.length - 1; i++) {
            if (order[i] == 2 && order[i + 1] == 1) {
                order[i + 1] = 0;
            }
        }
        return order;
    }
    public static int[] Order;
    // === Helper: Build the actual firing sequence of 3 balls based on color order ===
    public static Action shootSequence(AtomicBoolean shotReqFR, AtomicBoolean shotReqBL, AtomicBoolean intakeAtomic,
                                       char[] queue, int obeliskID) {

        char[] desired = getDesiredPattern(obeliskID);
        Order = computeFireOrder(queue, desired);

        ArrayList<Action> actions = new ArrayList<>();
        for (int i = 0; i <= 2; i++) {
            int feeder = Order[i]; // Go through the firing order (eg. [0, 1, 2]) and set shot requests to true
            //TODO: modify for an intake pulse when firing slot 2. This will cause slot 1 to be displaced to slot 0. (push it in one slot)
            if (feeder == 2) {
                actions.add(new InstantAction(() -> intakeAtomic.set(true)));
                actions.add(new SleepAction(0.12));
                actions.add(new InstantAction(() -> intakeAtomic.set(false)));
                actions.add(new SleepAction(0.08));
            }
            // then set shotReq booleans and the usual sleep/reset sequence
            actions.add(new InstantAction(() -> {
                if (feeder == 1 || feeder == 2) shotReqFR.set(true);  // 1, 2 = front/right feeder
                else shotReqBL.set(true);               // 0 = back/left feeder
            }));
            actions.add(new SleepAction(0.2)); // Wait between shots
            actions.add(new InstantAction(() -> {
                shotReqFR.set(false);
                shotReqBL.set(false);
            }));
            actions.add(new SleepAction(0.8)); // Wait between shots
        }
        SequentialAction seq = new SequentialAction(actions);
        return seq;
    }
}