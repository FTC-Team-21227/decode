package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.Collection;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;


@Autonomous(name = "Auto_Red_Goal")
public class Auto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.Color color = Robot.Color.RED;
        boolean cont = true;
        while (cont && !isStopRequested()){
            if (gamepad1.bWasPressed()){
                color = Robot.Color.RED;
            }
            if (gamepad1.xWasPressed()){
                color = Robot.Color.BLUE;
            }
            if (gamepad1.startWasPressed()){
                cont = false;
            }
            telemetry.addData("Color", color);
            telemetry.addLine("b = red, x = blue, start = continue");
            telemetry.update();
        }
        double row = 0;
        cont = true;
        while (cont && !isStopRequested()){
            if (gamepad1.aWasPressed()){
                row = 0;
            }
            if (gamepad1.bWasPressed()){
                row = 1;
            }
            if (gamepad1.yWasPressed()){
                row = 2;
            }
            if (gamepad1.startWasPressed()){
                cont = false;
            }
            telemetry.addData("Color", color);
            telemetry.addData("Row Num ",row);
            telemetry.addLine("a = 0 (near goal), b = 1 (middle), y = 2 (near loading), start = continue");
            telemetry.update();
        }

        telemetry.addData("Color", color);
        telemetry.addData("Row Num", row);
        telemetry.addLine("If incorrect, stop and reinit");
        telemetry.update();

//        Pose2d initialPose = new Pose2d(-55, 46, Math.toRadians(-55));
        Pose2d initialPose = new Pose2d(-57.25, 46, Math.toRadians(-50));
        Robot robot = Robot.startInstance(initialPose, color /*Robot.Color.RED*/);
        robot.initAuto(hardwareMap, telemetry);
        MecanumDrive drive = robot.drive2;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose) //first specimen
                .strafeTo(new Vector2d(-12,15)) // Shooting pos
                .turnTo(Math.toRadians(180)) // Face obelisk
                ;
        TrajectoryActionBuilder tab = drive.actionBuilder(new Pose2d(-12,15,Math.toRadians(180))) //first specimen
                .turnTo(Math.toRadians(90)) // Face the row of artifacts
                ;
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-12,15,Math.toRadians(90))) //first specimen
                .strafeTo(new Vector2d(-12,49), new TranslationalVelConstraint(15)) // Collect closest row of artifacts
                .strafeTo(new Vector2d(-12,15)) // Back up to shooting pos
                ;
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(-12,15,Math.toRadians(90)))
                .strafeTo(new Vector2d(15,15)) // Strafe right to next row of artifacts
                .strafeTo(new Vector2d(15,51), new TranslationalVelConstraint(15)) // Collect artifacts
                .strafeTo(new Vector2d(-12,15)) // Shooting pos
                ;
        TrajectoryActionBuilder tab4Mysterious = drive.actionBuilder(new Pose2d(-12,15,Math.toRadians(90)))
                .strafeTo(new Vector2d(36,15))
                .strafeTo(new Vector2d(36,50), new TranslationalVelConstraint(15))
                .strafeTo(new Vector2d(-12,15))
                ;
        TrajectoryActionBuilder parktab = drive.actionBuilder(new Pose2d(-12,15,Math.toRadians(90)))
                .strafeTo(new Vector2d(-6,15)) // Strafe to parking
                ;

        ArrayList<TrajectoryActionBuilder> trajs = new ArrayList<>();
        trajs.add(tab1); trajs.add(tab); trajs.add(tab2); trajs.add(tab3); trajs.add(parktab); trajs.add(tab4Mysterious);
        ArrayList<char[]> queues = new ArrayList<>();
        queues.add(new char[]{'P','P','G'}); queues.add(new char[]{'P','G','P'}); queues.add(new char[]{'G','P','P'});

        if (row == 1){
            //trajs swap tab2 and tab3
            TrajectoryActionBuilder t = trajs.get(2);
            trajs.set(2,trajs.get(3));
            trajs.set(3, t); //doesn't matter in 9 ball but matters in 12 ball
            //queues swap 0 and 1
            char[] q = queues.get(0);
            queues.set(0,queues.get(1));
            queues.set(1,q);
        }
        if (row == 2){
            //trajs swap tab2 and tab4Mysterious
            TrajectoryActionBuilder t = trajs.get(2);
            trajs.set(2,trajs.get(5));
            trajs.set(5, t); //doesn't matter in 9 ball but matters in 12 ball
            //queues swap 0 and 2
            char[] q = queues.get(0);
            queues.set(0,queues.get(2));
            queues.set(2,q);
        }

        //by default: build all except tab4mysterious
        Action firstTrajectory = trajs.get(0).build();
        Action turnGoal = trajs.get(1).build();
        Action secondTrajectory = trajs.get(2).build();
        Action thirdTrajectory = trajs.get(3).build();
        Action parkTrajectory = trajs.get(4).build();

        // Keeps track of robot's internal ball order based on what is intook first
        AtomicReference<char[]> currentQueue = new AtomicReference<>(new char[]{'P','G','P'}); // Intake order: purple, green, purple

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
                            Robot.shootSequence(shotReqFR, shotReqBL, intake, currentQueue.get(), id.get()),
                            new InstantAction(() -> intake.set(true)),

                            // COLLECT ROUND 2 BALLS
                            secondTrajectory,
                            new InstantAction(() -> intake.set(false)),
                            new InstantAction(() -> reverseIntake.set(true)),
                            new SleepAction(0.2),
                            new InstantAction(() -> reverseIntake.set(false)),
                            new SleepAction(0.2),
                                // Update internal ball order after intaking. ORDER: first intook = 0 idx.
                            new InstantAction(() -> {
                                // Example: assume collected PPG for second round
//                                currentQueue[0] = 'P';
//                                currentQueue[1] = 'P';
//                                currentQueue[2] = 'G';
                                currentQueue.set(queues.get(0));
                            }),

                            // FIRE ROUND 2
                            Robot.shootSequence(shotReqFR, shotReqBL, intake, currentQueue.get(), id.get()),
                            new InstantAction(() -> intake.set(true)),

                            // COLLECT ROUND 3 BALLS
                            thirdTrajectory,
                            new InstantAction(() -> intake.set(false)),
                            new InstantAction(() -> reverseIntake.set(true)),
                            new SleepAction(0.2),
                            new InstantAction(() -> reverseIntake.set(false)),
                            new SleepAction(0.2),

                                // Example: assume collected PPG for last round
                            new InstantAction(() -> {
//                                currentQueue[0] = 'P';
//                                currentQueue[1] = 'G';
//                                currentQueue[2] = 'P';
                                currentQueue.set(queues.get(1));
                            }),

                            Robot.shootSequence(shotReqFR, shotReqBL, intake, currentQueue.get(), id.get()),
                            new InstantAction(() -> intake.set(false)),

                            parkTrajectory
                            // END
                    ),
                    telemetryPacket -> {
                        robot.controlIntake(intake.get(), reverseIntake.get(), !intake.get(), false);
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
