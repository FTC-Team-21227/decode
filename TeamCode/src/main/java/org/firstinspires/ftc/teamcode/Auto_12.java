package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;


@Autonomous(name = "Auto_Red_Goal_12")
public class Auto_12 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-55, 46, Math.toRadians(-55));
        Robot robot = Robot.getInstance(hardwareMap, initialPose, Robot.Color.RED);
        robot.initAuto(telemetry);
        MecanumDrive drive = robot.drive2;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose) //first specimen
                .strafeTo(new Vector2d(-12,15))
                .turnTo(Math.toRadians(180))
                ;
        TrajectoryActionBuilder tab = drive.actionBuilder(new Pose2d(-12,15,Math.toRadians(180))) //first specimen
                .turnTo(Math.toRadians(90))
                ;
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-12,15,Math.toRadians(90))) //first specimen
                .strafeTo(new Vector2d(-12,45))
                .strafeTo(new Vector2d(-12,15))
                ;
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(-12,15,Math.toRadians(90)))
                .strafeTo(new Vector2d(12,15))
                .strafeTo(new Vector2d(12,45))
                .strafeTo(new Vector2d(-12,15))
                ;
        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(-12,15,Math.toRadians(90)))
                .strafeTo(new Vector2d(36,15))
                .strafeTo(new Vector2d(36,45))
                .strafeTo(new Vector2d(-12,15))
                ;

        Action firstTrajectory = tab1.build();
        Action turnGoal = tab.build();
        Action secondTrajectory = tab2.build();
        Action thirdTrajectory = tab3.build();
        Action fourthTrajectory = tab4.build();

        // Keeps track of robot's internal ball order based on what is intook first
        char[] currentQueue = {'P','G','P'}; // Intake order: purple, green, purple

//        Action p2p = robot.drive2.p2pAction();
        waitForStart();

        AtomicBoolean shotReqFR = new AtomicBoolean(false);
        AtomicBoolean shotReqBL = new AtomicBoolean(false);
        AtomicBoolean intake = new AtomicBoolean(false); // Intake on
        AtomicInteger id = new AtomicInteger(21); // Obelisk AprilTag ID #
        AtomicBoolean detectOb = new AtomicBoolean(false);
        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                            firstTrajectory,
                            new InstantAction(() -> detectOb.set(true)),
                            new SleepAction(0.5),
                            new InstantAction(() -> detectOb.set(false)),
                            turnGoal, // Turn 90 degrees
                            // FIRE ROUND 1 (detect obelisk)
                            Robot.shootSequence(shotReqFR, shotReqBL, intake, currentQueue, id.get()),
                            new InstantAction(() -> intake.set(true)),

                            // COLLECT ROUND 2 BALLS
                            secondTrajectory,
                            new InstantAction(() -> intake.set(false)),

                            // Update internal ball order after intaking. ORDER: first intook = 0 idx.
                            new InstantAction(() -> {
                                // Example: assume collected PPG for second round
                                currentQueue[0] = 'P';
                                currentQueue[1] = 'P';
                                currentQueue[2] = 'G';
                            }),

                            // FIRE ROUND 2
                            Robot.shootSequence(shotReqFR, shotReqBL, intake, currentQueue, id.get()),
                            new InstantAction(() -> intake.set(true)),

                            // COLLECT ROUND 3 BALLS
                            thirdTrajectory,
                            new InstantAction(() -> intake.set(false)),

                            // Example: assume collected PPG for last round
                            new InstantAction(() -> {
                                currentQueue[0] = 'P';
                                currentQueue[1] = 'G';
                                currentQueue[2] = 'P';
                            }),

                            // FIRE ROUND 3
                            Robot.shootSequence(shotReqFR, shotReqBL, intake, currentQueue, id.get()),
                            new InstantAction(() -> intake.set(true)),

                            // COLLECT ROUND 3 BALLS
                            fourthTrajectory,
                            new InstantAction(() -> intake.set(false)),

                            // Example: assume collected PPG for last round
                            new InstantAction(() -> {
                                currentQueue[0] = 'G';
                                currentQueue[1] = 'P';
                                currentQueue[2] = 'P';
                            }),

                            // FIRE ROUND 3
                            Robot.shootSequence(shotReqFR, shotReqBL, intake, currentQueue, id.get())
                    ),
                    telemetryPacket -> {
                        robot.controlIntake(intake.get(), false, !intake.get());
                        robot.updateShooter(shotReqFR.get(), shotReqBL.get(), false, telemetry, true, Robot.Constants.autoShotPose, 0,false,false,false, false, false);
                        id.set(robot.camera.detectObelisk(telemetry, detectOb.get()));
                        telemetry.addData("Obelisk Detected", id.get());
                        int[] order = Robot.Order;
                        telemetryPacket.put("order", order[0] + ", " + order[1] + ", " + order[2]); // Feeder order
                        return true;
                    }

                )
        );
    }
}
