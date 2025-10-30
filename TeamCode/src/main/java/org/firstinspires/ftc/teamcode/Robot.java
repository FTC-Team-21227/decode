package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
        double Motor_fwd_power = Math.cos(Heading_Angle / 180 * Math.PI) * Motor_FWD_input - Math.sin(Heading_Angle / 180 * Math.PI) * Motor_Side_input;
        double Motor_side_power = (Math.cos(Heading_Angle / 180 * Math.PI) * Motor_Side_input + Math.sin(Heading_Angle / 180 * Math.PI) * Motor_FWD_input) *MecanumDrive.PARAMS.inPerTick/ MecanumDrive.PARAMS.lateralInPerTick; //*1.5
        double Motor_Rotation_power = rotate * 0.7; //0.5
        double Motor_power_BL = -(((Motor_fwd_power - Motor_side_power) - Motor_Rotation_power) * Constants.drivePower);
        double Motor_power_BR = -(((Motor_fwd_power + Motor_side_power) + Motor_Rotation_power) * Constants.drivePower);
        double Motor_power_FL = -(((Motor_fwd_power + Motor_side_power) - Motor_Rotation_power) * Constants.drivePower);
        double Motor_power_FR = -(((Motor_fwd_power - Motor_side_power) + Motor_Rotation_power) * Constants.drivePower);
        drive2.leftFront.setPower(Motor_power_FL);
        drive2.rightFront.setPower(Motor_power_FR);
        drive2.leftBack.setPower(Motor_power_BL);
        drive2.rightBack.setPower(Motor_power_BR);
    }

    // Calculates and sets hood angle and flywheel RPM. Includes shooter state manager.
    public void updateShooter(boolean shotRequestedFront, boolean shotRequestedBack, Telemetry telemetry, boolean setPose, Pose2d setRobotPose, boolean humanFeed) { // TODO: add some params where u can hardset shooter outputs
        // TODO: replace these with LUT values
        // Assume we have: Vector2d goalPosition
        Pose2d poseRobot = drive2.localizer.getPose();
//        Pose2d pose = new Pose2d(poseRobot.position.plus(Constants.turretPos), poseRobot.heading); // Pose with the TURRET's position and ROBOT's heading
        Pose2d pose = poseRobot.times(Constants.turretPos); // Pose with the TURRET's position and ROBOT's heading
        if (setPose){
            pose = setRobotPose;
        }
        Vector2d goalVector = Constants.goalPos.minus(pose.position);


        double p = 0.75; // Fraction of time along trajectory from ground to ground
        double g = 386.22; // Gravity (in/s^2)

        double deltaH = Constants.deltaH; // Height difference from shooter to goal
        double distance = goalVector.norm(); // Horizontal distance

        double flightTime = Math.sqrt(2 * deltaH / (p * g * (1 - p))); // Ball trajectory time from ground to ground
        double theta = Math.atan(deltaH / (distance * (1 - p))); // Ball launch angle of elevation
        double vel = distance / (p * flightTime * Math.cos(theta)); // Ball launch speed
//        double absoluteAngleToGoal = /*Math.PI + */Constants.goalPos.minus(pose.position).angleCast().toDouble();
        double goalVectorAngle = goalVector.angleCast().toDouble();
        double heading = pose.heading.toDouble();
        double turretAngle = goalVectorAngle- heading; // Angle to turn turret to (relative to robot's heading)
        double wheelRadius = 1.89; // Inches, for example
        /*
        double wheelCircumference = Math.PI * wheelDiameter;
        double change = 0;
        if (up) change += 0.001;
        if (down) change -= 0.001;
        StarterRobot.Constants.flywheelPower += change;
         */

        // Convert vel (speed) to rad/s (example calibration: vel = wheelRadius * rad/s
        double radps = vel / wheelRadius * Constants.flywheelPower; // RPM
        turret.turnToRobotAngle(turretAngle);
        // Set flywheel RPM
        // TODO: UNCOMMENT LATER!
        if (humanFeed){
            flywheel.spinTo(-800);
        }
        else flywheel.spinTo(radps * 28 / Math.PI / 2);
        // Set hood angle to theta (convert to servo position)
        hood.turnToAngle(theta);

        switch (launchState) {
            case IDLE:
                if (shotRequestedFront) {
                    if (feederTimer.seconds()>Constants.feedTime) // After feeding is done
                        launchState = LaunchState.SPIN_UP_FRONT;
                }
                else if (shotRequestedBack){
                    if (feederTimer.seconds()>Constants.feedTime)
                        launchState = LaunchState.SPIN_UP_BACK;
                }
                break;
            case SPIN_UP_FRONT: // SPEED UP FLYWHEEL
                if (flywheel.getVel() > radps * 28 / Math.PI / 2 - 50) {
                    launchState = LaunchState.FEED_FRONT;
                }
//                launchState = launchState.FEED_FRONT;
                break;
            case SPIN_UP_BACK: // SPEED UP FLYWHEEL
                if (flywheel.getVel() > radps * 28 / Math.PI / 2 - 50) {
                    launchState = LaunchState.FEED_BACK;
                }
//                launchState = launchState.FEED_BACK;
                break;
            case FEED_FRONT: // FEED BALL
                feeder.upFR(); // feeder starts
                feederTimer.reset(); // feeder goes down
                launchState = LaunchState.LAUNCHING;
                break;
            case FEED_BACK: // FEED BALL
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
                }
                break;
        }

        // TELEMETRY LINES
        if (hood.commandedOutsideRange()) telemetry.addLine("WARNING: hood commanded out of its range! Auto set to 0 or 1.");
        if (turret.commandedOutsideRange()) telemetry.addLine("WARNING: turret commanded out of its range! Auto set to 0 or 1.");
        telemetry.addData("flywheel power scale factor", Constants.flywheelPower);
        telemetry.addData("State", launchState);
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
    public void updateLocalizer(Telemetry telemetry){
        switch (driveState){
            case RELATIVE:
                drive2.updatePoseEstimate();
//                if (aprilTimer.seconds() > 10){
//                    driveState = DriveState.ABSOLUTE;
//                }
                break;
            case ABSOLUTE:
                drive2.relocalize(telemetry);
                aprilTimer.reset();
                driveState = DriveState.RELATIVE;
                break;
        }
        telemetry.addData("x", drive2.localizer.getPose().position.x);
        telemetry.addData("y", drive2.localizer.getPose().position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive2.localizer.getPose().heading.toDouble()));
    }

    public void controlIntake(boolean in, boolean out, boolean stop){
        if (in) intake.intake();
        else if (out) intake.outtake();
        else if (stop) intake.stop();
    }
    // Lookup table (lut)
    public int[][][] power = {{{1}}};
    public int[][][] angle = {{{1}}};
    public int[] lookUp(Vector2d pos, Vector2d vel){

        return new int[]{power[0][0][0],angle[0][0][0]};
    }

    public Pose2d mirrorPose(Pose2d pose){
        return new Pose2d(new Vector2d(pose.position.x, -pose.position.y), new Rotation2d(pose.heading.real, -pose.heading.imag));
    }
}