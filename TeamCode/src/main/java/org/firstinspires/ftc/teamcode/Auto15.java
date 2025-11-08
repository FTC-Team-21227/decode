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

// FROM BLUE LINE 15 MEEP MEEP
@Autonomous(name = "Auto_Red_15")
public class Auto15 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-55, 46, Math.toRadians(-55));
        Robot robot = Robot.getInstance(hardwareMap, initialPose, Robot.Color.RED);
        robot.initAuto(telemetry);
        MecanumDrive drive = robot.drive2;


        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                // 1st Trajectory: Move out, read obelisk
                .strafeTo(new Vector2d(-12, 46 - 20 * Math.tan(Math.toRadians(55))))
                .turnTo(Math.toRadians(180))
                // TODO: Shoot here. 3/15
                ;
        TrajectoryActionBuilder tab = drive.actionBuilder(new Pose2d(-12,15,Math.toRadians(180)))
                // Face artifacts
                .turnTo(Math.toRadians(90))
                ;
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-12,15,Math.toRadians(90)))
                // 2nd Trajectory: collect 2nd row and open gate
                .strafeTo(new Vector2d(12, 30)) // Line up with artifacts
                .strafeTo(new Vector2d(12, 45)) // Move forward to collect
                .strafeTo(new Vector2d(10, 56)) // Open gate
                .strafeTo(new Vector2d(8, 40)) // Back up (avoid hitting 1st row)
                .strafeTo(new Vector2d(-12, 15)) // Shooting pos
                // TODO: Shoot here. 6/15
                ;
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(-12,15,Math.toRadians(90)))
                // 3rd Trajectory: collect 1st row
                .strafeTo(new Vector2d(-12, 45)) // Move forward to collect
                .strafeTo(new Vector2d(-12, 15)) // Shooting pos
                // TODO: Shoot here. 9/15
                ;
        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(-12,15,Math.toRadians(90)))
                // 4th Trajectory: collect 3rd (last) row
                .strafeTo(new Vector2d(36, 30)) // Line up with artifacts
                .strafeTo(new Vector2d(36, 45)) // Move forward to collect
                .strafeTo(new Vector2d(-12, 15)) // Shooting pos
                // TODO: Shoot here. 12/15
                ;
        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(-12,15,Math.toRadians(90)))
                // 5th Trajectory: collect 3 from gate release
                .strafeTo(new Vector2d(20, 58)) // Collect from gate release
                .strafeTo(new Vector2d(-12, 15)) // Shooting pos
                // TODO: Shoot here. 15/15
                ;
        TrajectoryActionBuilder tabp = drive.actionBuilder(new Pose2d(-12,15,Math.toRadians(90)))
                .strafeTo(new Vector2d(40,26.5)) // Strafe to parking
                ;
        Action parkTrajectory = tabp.build();

        Action firstTrajectory = tab1.build();
        Action turnGoal = tab.build();
        Action secondTrajectory = tab2.build();
        Action thirdTrajectory = tab3.build();
        Action fourthTrajectory = tab4.build();
        Action fifthTrajectory = tab5.build();

        // Keeps track of robot's internal ball order based on what is intook first
        char[] currentQueue = {'P','G','P'}; // Intake order: purple, green, purple

//        Action p2p = robot.drive2.p2pAction();
        waitForStart();

        AtomicBoolean shotReqFR = new AtomicBoolean(false);
        AtomicBoolean shotReqBL = new AtomicBoolean(false);
        AtomicBoolean intake = new AtomicBoolean(false); // Intake on
        AtomicBoolean reverseIntake = new AtomicBoolean(false); // Intake on
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
                            // COLLECT ROW 2 BALLS
                            secondTrajectory,
                            new InstantAction(() -> intake.set(false)),
                            new InstantAction(() -> reverseIntake.set(true)),
                            new SleepAction(0.1),
                            new InstantAction(() -> reverseIntake.set(false)),
                            // Update internal ball order after intaking. ORDER: first intook = 0 idx.
                            new InstantAction(() -> { // ROW 2 ORDER
                                currentQueue[0] = 'P';
                                currentQueue[1] = 'G';
                                currentQueue[2] = 'P';
                            }),
                            // FIRE ROUND 2
                            Robot.shootSequence(shotReqFR, shotReqBL, intake, currentQueue, id.get()),
                            new InstantAction(() -> intake.set(true)),
                            // COLLECT ROW 1 BALLS
                            thirdTrajectory,
                            new InstantAction(() -> intake.set(false)),
                            new InstantAction(() -> reverseIntake.set(true)),
                            new SleepAction(0.1),
                            new InstantAction(() -> reverseIntake.set(false)),
                            new InstantAction(() -> { // ROW 1 ORDER
                                currentQueue[0] = 'P';
                                currentQueue[1] = 'P';
                                currentQueue[2] = 'G';
                            }),
                            // FIRE ROUND 3
                            Robot.shootSequence(shotReqFR, shotReqBL, intake, currentQueue, id.get()),
                            new InstantAction(() -> intake.set(true)),
                            // COLLECT ROW 3 BALLS
                            fourthTrajectory,
                            new InstantAction(() -> intake.set(false)),
                            new InstantAction(() -> reverseIntake.set(true)),
                            new SleepAction(0.1),
                            new InstantAction(() -> reverseIntake.set(false)),
                            new InstantAction(() -> { // ROW 3 ORDER
                                currentQueue[0] = 'G';
                                currentQueue[1] = 'P';
                                currentQueue[2] = 'P';
                            }),
                            // FIRE ROUND 4
                            Robot.shootSequence(shotReqFR, shotReqBL, intake, currentQueue, id.get()),
                            new InstantAction(() -> intake.set(true)),

                            parkTrajectory
                            // COLLECT GATE RELEASE BALLS
//                            fifthTrajectory,
//                            new InstantAction(() -> intake.set(false)),
//                            new InstantAction(() -> { // ORDER UNKNOWN
//                                currentQueue[0] = 'G';
//                                currentQueue[1] = 'P';
//                                currentQueue[2] = 'P';
//                            }),
                            // FIRE ROUND 5
//                            Robot.shootSequence(shotReqFR, shotReqBL, intake, currentQueue, id.get()),
//                            new InstantAction(() -> intake.set(true))
                    ),
                    telemetryPacket -> {
                        robot.controlIntake(intake.get(), false, !intake.get(), false);
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
