package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class StarterRobot {
    Feeders feeders;
    Flywheel flywheel;
    public Camera camera;

    //initialize subsystems
    public StarterRobot(HardwareMap hardwareMap){
        feeders = new Feeders(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
//        camera = new Camera(hardwareMap);
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

        telemetry.addData("Status", "Initialized");
    }

    //constants
    @Config
    public static class Constants{
        public final static Vector2d turretPos = new Vector2d(0,-5);
        public final static double p = 300, i = 0, d = 0, f = 10;
        public final static double feederPower = 1.0;
        public final static double intakePower = 1.0;
        public final static double outtakePower = -1.0;
        public final static double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
        public final static double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
        public final static double FULL_SPEED = 1.0;
        public final static double LAUNCHER_TARGET_VELOCITY = 100;
        public final static double LAUNCHER_MIN_VELOCITY = 90;

    }

    // Thanks to FTC16072 for sharing this code!!


    //queuer/state machine
    public void updateShooter(boolean shotRequested, Telemetry telemetry) {
        //replace these with LUT values
        flywheel.spinTo(Constants.LAUNCHER_TARGET_VELOCITY);
        //-/-///
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
//                flywheel.spinTo(Constants.LAUNCHER_TARGET_VELOCITY);
                if (flywheel.getVel() > Constants.LAUNCHER_MIN_VELOCITY) {
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
        telemetry.addData("State", launchState);
        telemetry.addData("motorSpeed", flywheel.getVel());
    }

    public Pose2d updateLocalizer(){
        switch (driveState){
            case ABSOLUTE:
                Pose2d pose = camera.update();
                pose = new Pose2d(pose.position.minus(Robot.Constants.turretPos),pose.heading.toDouble()-0);
                return pose;
//                driveState = DriveState.RELATIVE;
        }
        return null;
    }
}