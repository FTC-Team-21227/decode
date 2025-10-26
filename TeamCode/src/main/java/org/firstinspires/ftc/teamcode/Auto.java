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

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;


@Autonomous(name = "Auto_Red_Goal")
public class Auto extends LinearOpMode {
    // === Helper: Determine desired color sequence per obelisk ID ===
    private char[] getDesiredPattern(int obeliskID) {
        switch (obeliskID) {
            case 21: return new char[]{'G','P','P'}; // left
            case 22: return new char[]{'P','G','P'}; // center
            case 23: return new char[]{'P','P','G'}; // right
            default: return new char[]{'G','P','P'}; // fallback
        }
    }

    // === Helper: Compute mapping from internal queue → desired order ===
    private int[] computeFireOrder(char[] queue, char[] desired) {
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
        return order;
    }

    // === Helper: Build the actual firing sequence based on color order ===
    private Action shootSequence(AtomicBoolean shotReqFR, AtomicBoolean shotReqBL,
                                 char[] queue, int obeliskID) {

        char[] desired = getDesiredPattern(obeliskID);
        int[] order = computeFireOrder(queue, desired);

        ArrayList<Action> actions = new ArrayList<>();
        for (int i = 0; i <= 2; i++) {
            int feeder = order[i];
            actions.add(new InstantAction(() -> {
                if (feeder == 1) shotReqFR.set(true);  // 1 = front/right feeder
                else shotReqBL.set(true);               // 0, 2 = back/left feeder
            }));
            actions.add(new SleepAction(1)); // time to shoot
            actions.add(new InstantAction(() -> {
                shotReqFR.set(false);
                shotReqBL.set(false);
            }));
        }
        SequentialAction seq = new SequentialAction(actions);
        return seq;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-55, 46, Math.toRadians(-55));
        Robot robot = Robot.getInstance(hardwareMap, initialPose, Robot.Color.RED);
        robot.initAuto(telemetry);
        MecanumDrive drive = robot.drive2;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose) //first specimen
                .strafeTo(new Vector2d(-12,46-20*Math.tan(Math.toRadians(55))))
                .turnTo(Math.toRadians(180))
                ;
        TrajectoryActionBuilder tab = drive.actionBuilder(new Pose2d(-12,46-20*Math.tan(Math.toRadians(55)),Math.toRadians(180))) //first specimen
                .turnTo(Math.toRadians(90))
                ;
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-12,46-20*Math.tan(Math.toRadians(55)),Math.toRadians(90))) //first specimen
                .strafeTo(new Vector2d(-12,45))
                .strafeTo(new Vector2d(-12,15))
                ;
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(-12,15,Math.toRadians(90))) //first specimen
                .strafeTo(new Vector2d(12,15))
                .strafeTo(new Vector2d(12,45))
                .strafeTo(new Vector2d(-12,15))
                ;

        Action firstTrajectory = tab1.build();
        Action turnGoal = tab.build();
        Action secondTrajectory = tab2.build();
        Action thirdTrajectory = tab3.build();

        char[] currentQueue = {'P','G','P'}; // Example: purple front, green middle, purple back

//        Action p2p = robot.drive2.p2pAction();
        waitForStart();

        AtomicBoolean shotReqFR = new AtomicBoolean(false);
        AtomicBoolean shotReqBL = new AtomicBoolean(false);
        AtomicBoolean intake = new AtomicBoolean(false);
        AtomicInteger id = new AtomicInteger(21);
        AtomicBoolean detectOb = new AtomicBoolean(false);
//        shotReq.set(false);
//        Actions.runBlocking(
//                new ParallelAction(
//                        new SequentialAction(
////                            new SleepAction(5), //wait for flywheel to charge up
//                            //give 3 seconds to shoot the first set of 3 balls
//                            firstTrajectory, //drive to shot spot
//                            new InstantAction(() -> detectOb.set(true)),
//                            new SleepAction(0.5),
//                            turnGoal,
//                            new InstantAction(() -> shotReqFR.set(true)),
//                            new SleepAction(1),
//                            new InstantAction(() -> shotReqFR.set(false)),
//                            new InstantAction(() -> shotReqBL.set(true)),
//                            new SleepAction(1),
//                            new InstantAction(() -> shotReqBL.set(false)),
//                            new InstantAction(() -> shotReqFR.set(true)),
//                            new SleepAction(1),
//                            new InstantAction(() -> shotReqFR.set(false)),
//                            new InstantAction(() -> intake.set(true)),
////                            new InstantAction(() -> shotReq.set(false)),
//                            secondTrajectory,
//                            //give 3 seconds to shoot the second set of 3 balls
//                            new InstantAction(() -> intake.set(false)),
////                            new InstantAction(() -> shotReq.set(true)),
////                            new SleepAction(3),
//                            new InstantAction(() -> shotReqFR.set(true)),
//                            new SleepAction(1),
//                            new InstantAction(() -> shotReqFR.set(false)),
//                            new InstantAction(() -> shotReqBL.set(true)),
//                            new SleepAction(1),
//                            new InstantAction(() -> shotReqBL.set(false)),
//                            new InstantAction(() -> shotReqFR.set(true)),
//                            new SleepAction(1),
//                            new InstantAction(() -> shotReqFR.set(false)),
//                            new InstantAction(() -> intake.set(true)),
////                            new InstantAction(() -> shotReq.set(false)),
//                            thirdTrajectory, //gobble up row 2
//                            new InstantAction(() -> intake.set(false)),
//                            //give 3 seconds to shoot the third set of 3 balls
////                            new InstantAction(() -> shotReq.set(true)),
////                            new SleepAction(3),
////                            new InstantAction(() -> shotReq.set(false))
//                            new InstantAction(() -> shotReqFR.set(true)),
//                            new SleepAction(1),
//                            new InstantAction(() -> shotReqFR.set(false)),
//                            new InstantAction(() -> shotReqBL.set(true)),
//                            new SleepAction(1),
//                            new InstantAction(() -> shotReqBL.set(false)),
//                            new InstantAction(() -> shotReqFR.set(true)),
//                            new SleepAction(1),
//                            new InstantAction(() -> shotReqFR.set(false))
//                        ),
//                        telemetryPacket -> {
//                            robot.controlIntake(intake.get(),false,!intake.get());
//                            robot.updateShooter(shotReqFR.get(),shotReqBL.get(),telemetry, true, Robot.Constants.autoShotPose, false);
//                            id.set(robot.camera.detectObelisk(telemetry,detectOb.get()));
//                            return true;
//                        }
//
//                )
//        );
        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                            firstTrajectory,
                            new InstantAction(() -> detectOb.set(true)),
                            new SleepAction(0.5),
                            new InstantAction(() -> detectOb.set(false)),
                            turnGoal,
                            // FIRE ROUND 1 (based on detected obelisk)
                            new InstantAction(() -> id.set(robot.camera.detectObelisk(telemetry, detectOb.get()))),
                            new InstantAction(() -> telemetry.addData("Obelisk Detected", id.get())),
                            shootSequence(shotReqFR, shotReqBL, currentQueue, id.get()),
                            new InstantAction(() -> intake.set(true)),

                            // COLLECT ROUND 2 BALLS
                            secondTrajectory,
                            new InstantAction(() -> intake.set(false)),

                            // Update internal ball order after intaking
                            new InstantAction(() -> {
                                // Example: assume collected GPP for second round
                                currentQueue[0] = 'G';
                                currentQueue[1] = 'P';
                                currentQueue[2] = 'P';
                            }),

                            shootSequence(shotReqFR, shotReqBL, currentQueue, id.get()),
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

                            shootSequence(shotReqFR, shotReqBL, currentQueue, id.get())
                    ),
                    telemetryPacket -> {
                        robot.controlIntake(intake.get(), false, !intake.get());
                        robot.updateShooter(shotReqFR.get(), shotReqBL.get(), telemetry, true, Robot.Constants.autoShotPose, false);
                        id.set(robot.camera.detectObelisk(telemetry, detectOb.get()));
                        return true;
                    }

                )
        );
    }
}
