package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;


@Autonomous(name = "Blue_Near_9")
public class Auto_Blue extends LinearOpMode {
    @Override
    // Choosing color and row
    public void runOpMode() throws InterruptedException {
        Robot.Color color = Robot.Color.BLUE;
        double row = 0;
        boolean cont = true;
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

        // Initialize robot, start position
        Pose2d initialPose = new Pose2d(-41.36, -54.62, Math.toRadians(180)); // Against blue wall
        Robot robot = Robot.startInstance(initialPose, color);
        robot.initAuto(hardwareMap, telemetry, Robot.OpModeState.AUTO);
        telemetry.update();
        MecanumDrive drive = robot.drive2;

        // Create trajectories
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                // Shooting pos
                .strafeTo(new Vector2d(-12,-15))
                // Face obelisk
//                .turnTo(Math.toRadians(180))
                ;
        TrajectoryActionBuilder tab = drive.actionBuilder(new Pose2d(-12,-15,Math.toRadians(180))) //first specimen
//                .turnTo(Math.toRadians(90)) // Face the row of artifacts
                .strafeToLinearHeading(new Vector2d(-41,-34),Math.toRadians(-87), new AngularVelConstraint(Math.PI/2), new ProfileAccelConstraint(-20,40));
                ;
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-41,-34,Math.toRadians(-87))) //first specimen
                .strafeToLinearHeading(new Vector2d(-12,-22), Math.toRadians(-90),new TranslationalVelConstraint(100), new ProfileAccelConstraint(-40,100))
                .strafeTo(new Vector2d(-12,-52), new TranslationalVelConstraint(15)) // Collect closest row of artifacts
                ;
        TrajectoryActionBuilder tab2_back = drive.actionBuilder(new Pose2d(-12,-52,Math.toRadians(-90))) //first specimen
                .strafeToLinearHeading(new Vector2d(-41,-34),Math.toRadians(-87)) // Back up to shooting pos
                ;
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(-41,-34,Math.toRadians(-87)))
                .strafeToLinearHeading(new Vector2d(12,-22),Math.toRadians(-90),new TranslationalVelConstraint(100), new ProfileAccelConstraint(-40,100)) // Strafe right to next row of artifacts
                .strafeTo(new Vector2d(12,-56), new TranslationalVelConstraint(15)) // Collect artifacts
                ;
        TrajectoryActionBuilder tab3_back = drive.actionBuilder(new Pose2d(15,-56,Math.toRadians(-90)))
                .strafeTo(new Vector2d(12,-34))
                .strafeToLinearHeading(new Vector2d(-41,-34),Math.toRadians(-87)) // Shooting pos
                ;
        TrajectoryActionBuilder tab4Mysterious = drive.actionBuilder(new Pose2d(-41,-34,Math.toRadians(-87)))
                .strafeToLinearHeading(new Vector2d(28,-22),Math.toRadians(90),new TranslationalVelConstraint(100), new ProfileAccelConstraint(-40,100))
                .strafeTo(new Vector2d(28,-56), new TranslationalVelConstraint(15))
                ;
        TrajectoryActionBuilder tab4_back = drive.actionBuilder(new Pose2d(28,-56,Math.toRadians(-90)))
                .strafeTo(new Vector2d(28,-34))
                .strafeToLinearHeading(new Vector2d(-41,-34),Math.toRadians(-87))
                ;
        TrajectoryActionBuilder parktab = drive.actionBuilder(new Pose2d(-41,-34,Math.toRadians(-87)))
                .strafeToLinearHeading(new Vector2d(-50,-20), Math.toRadians(-135),new TranslationalVelConstraint(100), new ProfileAccelConstraint(-40,100)) // Strafe to parking
                ;

        // Trajectory list
        ArrayList<TrajectoryActionBuilder> trajs = new ArrayList<>();
        trajs.add(tab1); trajs.add(tab); trajs.add(tab2); trajs.add(tab3); trajs.add(parktab); trajs.add(tab4Mysterious);
        ArrayList<TrajectoryActionBuilder> trajs_back = new ArrayList<>();
        trajs_back.add(tab2_back); trajs_back.add(tab3_back); trajs_back.add(tab4_back);
        // Ball order
        ArrayList<char[]> queues = new ArrayList<>();
        queues.add(new char[]{'P','P','G'}); queues.add(new char[]{'P','G','P'}); queues.add(new char[]{'G','P','P'});

        if (row == 1){
            // Swap tab2 and tab3
            TrajectoryActionBuilder t = trajs.get(2);
            trajs.set(2,trajs.get(3));
            trajs.set(3, t); // Doesn't matter in 9 ball but matters in 12 ball
            // Sack swap tab2back and tab3back
            TrajectoryActionBuilder tr = trajs_back.get(0);
            trajs_back.set(0,trajs_back.get(1));
            trajs_back.set(1, tr); // Doesn't matter in 9 ball but matters in 12 ball
            // Queues swap 0 and 1
            char[] q = queues.get(0);
            queues.set(0,queues.get(1));
            queues.set(1,q);
        }
        if (row == 2){
            // Swap tab2 and tab4Mysterious
            TrajectoryActionBuilder t = trajs.get(2);
            trajs.set(2,trajs.get(5));
            trajs.set(5, t); // Doesn't matter in 9 ball but matters in 12 ball
            // Swap tab2back and tab3back
            TrajectoryActionBuilder tr = trajs_back.get(0);
            trajs_back.set(0,trajs_back.get(2));
            trajs_back.set(2, tr); // Doesn't matter in 9 ball but matters in 12 ball
            // Queues swap 0 and 2
            char[] q = queues.get(0);
            queues.set(0,queues.get(2));
            queues.set(2,q);
        }

        // Default: Build all except tab4mysterious
        Action firstTrajectory = trajs.get(0).build();
        Action turnGoal = trajs.get(1).build();
        Action secondTrajectory = trajs.get(2).build();
        Action thirdTrajectory = trajs.get(3).build();
        Action parkTrajectory = trajs.get(4).build();
        Action secondTrajectory_back = trajs_back.get(0).build();
        Action thirdTrajectory_back = trajs_back.get(1).build();
        Action fourthTrajectory_back = trajs_back.get(2).build();

        // Keeps track of robot's internal ball order based on what is intook first
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
            // DETECT OBELISK ID
        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                            firstTrajectory,
                            new InstantAction(() -> detectOb.set(true)),
                            new SleepAction(0.2),
                            new InstantAction(() -> detectOb.set(false)),
                            new InstantAction(() -> con.set(false))
                        ),
                    telemetryPacket -> {
                        robot.controlIntake(intake.get(), reverseIntake.get(), stopIntake.get(), false, slowIntake.get(), false);
                        robot.updateShooter(shotReqFR.get(), shotReqBL.get(), false, telemetry, true, Robot.Positions.autoShotPose, 0,false,false,false, false, false, false, false, false,false, false, false);
                        // Detect and set obelisk ID
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
        Action firstShot = Robot.shootSequence(shotReqFR, shotReqBL, slowIntake, new char[]{'P','G','P'}, i, 1, Robot.OpModeState.AUTO, Robot.Color.BLUE);
        Action secondShot = Robot.shootSequence(shotReqFR, shotReqBL, slowIntake, queues.get(0), i, 2, Robot.OpModeState.AUTO, Robot.Color.BLUE);
        Action thirdShot = Robot.shootSequence(shotReqFR, shotReqBL, slowIntake, queues.get(1), i, 3, Robot.OpModeState.AUTO, Robot.Color.BLUE);
        Action fourthShot = Robot.shootSequence(shotReqFR, shotReqBL, slowIntake, queues.get(2), i, 4, Robot.OpModeState.AUTO, Robot.Color.BLUE);
        Actions.runBlocking(
                // REST OF THE TRAJECTORIES
                new ParallelAction(
                        new SequentialAction(
                                turnGoal, // Turn 90 degrees
                                // FIRE ROUND 1
                                firstShot,
                                new InstantAction(() -> intake.set(true)),

                                // COLLECT ROUND 2 BALLS
                                secondTrajectory,
                                new ParallelAction(
                                        secondTrajectory_back,
                                        new SequentialAction(
                                                new InstantAction(() -> intake.set(false)),
                                                new SleepAction(0.5),
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

                                // COLLECT ROUND 3 BALLS
                                thirdTrajectory,
                                new ParallelAction(
                                        thirdTrajectory_back,
                                        new SequentialAction(
                                                new InstantAction(() -> intake.set(false)),
                                                new SleepAction(0.5),
                                                new InstantAction(() -> reverseIntake.set(true)),
                                                new SleepAction(Robot.Constants.outtakePulseTime),
                                                new InstantAction(() -> reverseIntake.set(false)),
                                                new InstantAction(() -> stopIntake.set(true))
                                        )
                                ),

                                thirdShot,
                                new InstantAction(() -> intake.set(false)),

                                parkTrajectory
                                // END
                        ),
                        telemetryPacket -> {
                            robot.controlIntake(false/*intake.get()*/, reverseIntake.get(), stopIntake.get(), false, intake.get() || slowIntake.get(), false);
                            robot.updateShooter(shotReqFR.get(), shotReqBL.get(), false, telemetry, true, Robot.Positions.autoShotPose, 0,false,false,false,false, false, false, false, false, false, false, false);
//                            id.set(robot.camera.detectObelisk(telemetry, detectOb.get()));
                            telemetry.addData("Obelisk Detected", id.get());
                            int[] order = Robot.Order;
                            telemetryPacket.put("order", order[0] + ", " + order[1] + ", " + order[2]); // Feeder order
                            telemetry.update();
                            return true;
                        }

                )
        );}
        finally {
            robot.setPose(drive.localizer.getPose());
            RobotLog.a("Finally");
        }
    }
}
