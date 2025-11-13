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
import com.qualcomm.robotcore.util.RobotLog;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

// FROM BLUE LINE 15 MEEP MEEP
@Autonomous(name = "Near_GATE_12")
public class AutoNearGate12 extends LinearOpMode {
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

        telemetry.addData("Color", color);
        telemetry.addLine("If incorrect, stop and reinit");

        Pose2d initialPose = new Pose2d(-41.36, 55.81, Math.toRadians(180));
        Robot robot = Robot.startInstance(initialPose, color);
        robot.initAuto(hardwareMap, telemetry, Robot.OpModeState.AUTO);
        telemetry.update();
        MecanumDrive drive = robot.drive2;


        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                // 1st Trajectory: Move out, read obelisk
                .strafeTo(new Vector2d(-12, 15))
//                .turnTo(Math.toRadians(180))
                // TODO: Shoot here. 3/15
                ;
        TrajectoryActionBuilder tab = drive.actionBuilder(new Pose2d(-12,15,Math.toRadians(180)))
                // Face artifacts
                .turnTo(Math.toRadians(90))
                ;

        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-12,15,Math.toRadians(90)))
                // 3rd Trajectory: collect 1st row
                .strafeTo(new Vector2d(-7,30))
                .strafeTo(new Vector2d(-7,49), new TranslationalVelConstraint(15))
                // TODO: Shoot here. 9/15
                ;
        TrajectoryActionBuilder tab2_back = drive.actionBuilder(new Pose2d(-7,49,Math.toRadians(90))) //first specimen
                .strafeTo(new Vector2d(-7, 56)) // Open gate
                .strafeTo(new Vector2d(-12,15))
                ;
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(-12,15,Math.toRadians(90)))
                // 3rdd Trajectory: collect 2nd row and open gate
                .strafeTo(new Vector2d(15, 30)) // Line up with artifacts
                .strafeTo(new Vector2d(15,51), new TranslationalVelConstraint(15))
                // TODO: Shoot here. 6/15
                ;
        TrajectoryActionBuilder tab3_back = drive.actionBuilder(new Pose2d(15,51,Math.toRadians(90))) //first specimen
                .strafeTo(new Vector2d(-12,15))
                ;
        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(-12,15,Math.toRadians(90)))
                // 4th Trajectory: collect 3rd (last) row
                .strafeTo(new Vector2d(36, 30)) // Line up with artifacts
                .strafeTo(new Vector2d(36,50), new TranslationalVelConstraint(15))
                // TODO: Shoot here. 12/15
                ;
        TrajectoryActionBuilder tab4_back = drive.actionBuilder(new Pose2d(36,50,Math.toRadians(90))) //first specimen
                .strafeTo(new Vector2d(-12,15))
                ;
        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(-12,15,Math.toRadians(90)))
                // 5th Trajectory: collect 3 from gate release
                .strafeTo(new Vector2d(20, 58)) // Collect from gate release
                .strafeTo(new Vector2d(-12, 15)) // Shooting pos
                // TODO: Shoot here. 15/15
                ;
        TrajectoryActionBuilder tabp = drive.actionBuilder(new Pose2d(-12,15,Math.toRadians(90)))
                .strafeTo(new Vector2d(2,15)) // Strafe to parking
                ;
        Action parkTrajectory = tabp.build();

        Action firstTrajectory = tab1.build();
        Action turnGoal = tab.build();
        Action secondTrajectory = tab2.build();
        Action thirdTrajectory = tab3.build();
        Action fourthTrajectory = tab4.build();
        Action fifthTrajectory = tab5.build();
        Action secondTrajectory_back = tab2_back.build();
        Action thirdTrajectory_back = tab3_back.build();
        Action fourthTrajectory_back = tab4_back.build();

        // Keeps track of robot's internal ball order based on what is intook first
        char[] currentQueue = {'P','G','P'}; // Intake order: purple, green, purple

//        Action p2p = robot.drive2.p2pAction();
        waitForStart();

        AtomicBoolean shotReqFR = new AtomicBoolean(false);
        AtomicBoolean shotReqBL = new AtomicBoolean(false);
        AtomicBoolean intake = new AtomicBoolean(false); // Intake on
        AtomicBoolean slowIntake = new AtomicBoolean(false); // Intake on
        AtomicBoolean reverseIntake = new AtomicBoolean(false); // Intake on
        AtomicBoolean stopIntake = new AtomicBoolean(false); // Intake on
        AtomicInteger id = new AtomicInteger(21); // Obelisk AprilTag ID #
        AtomicBoolean detectOb = new AtomicBoolean(false);
        AtomicBoolean con = new AtomicBoolean(true);
try{
        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                            firstTrajectory,
                            new InstantAction(() -> detectOb.set(true)),
                            new SleepAction(0.5),
                            new InstantAction(() -> detectOb.set(false)),
                            new InstantAction(() -> con.set(false))
                        ),
                    telemetryPacket -> {
                        robot.controlIntake(intake.get(), reverseIntake.get(), stopIntake.get(), false, slowIntake.get());
                        robot.updateShooter(shotReqFR.get(), shotReqBL.get(), false, telemetry, true, Robot.Positions.autoShotPose, 0,false,false,false, false, false);
                        int od = (robot.camera.detectObelisk(telemetry, detectOb.get()));
                        if (od != 0) id.set(od);
//                            robot.camera.detectObelisk(telemetry, true);
                        telemetry.addData("Obelisk Detected", id.get());
                        int[] order = Robot.Order;
                        telemetryPacket.put("order", order[0] + ", " + order[1] + ", " + order[2]); // Feeder order
                        telemetry.update();
                        return con.get();
                    }

                )
        );
        robot.camera.close();
        int i = id.get();
        RobotLog.d("obelisk id", i);
        Action firstShot = Robot.shootSequence(shotReqFR, shotReqBL, slowIntake, new char[]{'P','G','P'}, i, 1);
        Action secondShot = Robot.shootSequence(shotReqFR, shotReqBL, slowIntake, new char[]{'P','P','G'}, i, 2);
        Action thirdShot = Robot.shootSequence(shotReqFR, shotReqBL, slowIntake, new char[]{'P','G','P'}, i, 3);
        Action fourthShot = Robot.shootSequence(shotReqFR, shotReqBL, slowIntake, new char[]{'G','P','P'}, i, 4);
        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                turnGoal, // Turn 90 degrees
                                // FIRE ROUND 1 (detect obelisk)
                                firstShot,
                                new InstantAction(() -> intake.set(true)),
                                // COLLECT ROW 2 BALLS
                                secondTrajectory,
                                new ParallelAction(
                                        secondTrajectory_back,
                                        new SequentialAction(
                                                new InstantAction(() -> intake.set(false)),
                                                new InstantAction(() -> reverseIntake.set(true)),
                                                new SleepAction(Robot.Constants.outtakePulseTime),
                                                new InstantAction(() -> reverseIntake.set(false)),
                                                new InstantAction(() -> stopIntake.set(true))
                                        )
                                ),
                                // FIRE ROUND 2
                                secondShot,
                                new InstantAction(() -> stopIntake.set(false)),
                                new InstantAction(() -> intake.set(true)),
                                // COLLECT ROW 1 BALLS
                                thirdTrajectory,
                                new ParallelAction(
                                        thirdTrajectory_back,
                                        new SequentialAction(
                                                new InstantAction(() -> intake.set(false)),
                                                new InstantAction(() -> reverseIntake.set(true)),
                                                new SleepAction(Robot.Constants.outtakePulseTime),
                                                new InstantAction(() -> reverseIntake.set(false)),
                                                new InstantAction(() -> stopIntake.set(true))
                                        )
                                ),
                                // FIRE ROUND 3
                                thirdShot,
                                new InstantAction(() -> stopIntake.set(false)),
                                new InstantAction(() -> intake.set(true)),
                                // COLLECT ROW 3 BALLS
                                fourthTrajectory,
                                new ParallelAction(
                                        fourthTrajectory_back,
                                        new SequentialAction(
                                                new InstantAction(() -> intake.set(false)),
                                                new InstantAction(() -> reverseIntake.set(true)),
                                                new SleepAction(Robot.Constants.outtakePulseTime),
                                                new InstantAction(() -> reverseIntake.set(false)),
                                                new InstantAction(() -> stopIntake.set(true))
                                        )
                                ),
                                // FIRE ROUND 4
                                fourthShot,
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
                            robot.controlIntake(intake.get(), reverseIntake.get(), stopIntake.get(), false, slowIntake.get());
                            robot.updateShooter(shotReqFR.get(), shotReqBL.get(), false, telemetry, true, Robot.Positions.autoShotPose, 0,false,false,false, false, false);
//                            id.set(robot.camera.detectObelisk(telemetry, detectOb.get()));
                            telemetry.addData("Obelisk Detected", id.get());
                            int[] order = Robot.Order;
                            telemetryPacket.put("order", order[0] + ", " + order[1] + ", " + order[2]); // Feeder order
                            telemetry.update();
                            return true;
                        }

                )
        );}
        finally{
            robot.setPose(drive.localizer.getPose());
            RobotLog.a("Finally");
        }
    }
}
