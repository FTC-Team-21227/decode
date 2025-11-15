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


@Autonomous(name = "Red_Near_9")
public class Auto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // SET ROBOT COLOR
        Robot.Color color = Robot.Color.RED;
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

//        Pose2d initialPose = new Pose2d(-55, 46, Math.toRadians(-55));
        Pose2d initialPose = new Pose2d(-41.36, 54.62, Math.toRadians(180));
        Robot robot = Robot.startInstance(initialPose, color /*Robot.Color.RED*/);
        robot.initAuto(hardwareMap, telemetry, Robot.OpModeState.AUTO);
        telemetry.update();
        MecanumDrive drive = robot.drive2;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose) //first specimen
                .strafeTo(new Vector2d(-12,15)) // Shooting pos
//                .turnTo(Math.toRadians(180)) // Face obelisk
                ;
        TrajectoryActionBuilder tab = drive.actionBuilder(new Pose2d(-12,15,Math.toRadians(180))) //first specimen
//                .turnTo(Math.toRadians(90)) // Face the row of artifacts
                .strafeToLinearHeading(Robot.Constants.autoShotPose.position, Robot.Constants.autoShotPose.heading.toDouble(), new AngularVelConstraint(Math.PI/2), new ProfileAccelConstraint(-20,40));
                ;
        TrajectoryActionBuilder tab2 = drive.actionBuilder(Robot.Constants.autoShotPose) //first specimen
                .strafeTo(new Vector2d(-12,22), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-40,100))
                .strafeTo(new Vector2d(-12,49), new TranslationalVelConstraint(15)) // Collect closest row of artifacts
                ;
        TrajectoryActionBuilder tab2_back = drive.actionBuilder(new Pose2d(-12,49,Math.toRadians(90))) //first specimen
                .strafeTo(Robot.Constants.autoShotPose.position) // Back up to shooting pos
                ;
        TrajectoryActionBuilder tab3 = drive.actionBuilder(Robot.Constants.autoShotPose)
                .strafeTo(new Vector2d(15,22),new TranslationalVelConstraint(100), new ProfileAccelConstraint(-40,100)) // Strafe right to next row of artifacts
                .strafeTo(new Vector2d(15,51), new TranslationalVelConstraint(15)) // Collect artifacts
                ;
        TrajectoryActionBuilder tab3_back = drive.actionBuilder(new Pose2d(15,51,Math.toRadians(90)))
                .strafeTo(Robot.Constants.autoShotPose.position) // Shooting pos
                ;
        TrajectoryActionBuilder tab4Mysterious = drive.actionBuilder(Robot.Constants.autoShotPose)
                .strafeTo(new Vector2d(30,22),new TranslationalVelConstraint(100), new ProfileAccelConstraint(-40,100))
                .strafeTo(new Vector2d(30,50), new TranslationalVelConstraint(15))
                ;
        TrajectoryActionBuilder tab4_back = drive.actionBuilder(new Pose2d(36,50,Math.toRadians(90)))
                .strafeTo(Robot.Constants.autoShotPose.position)
                ;
        TrajectoryActionBuilder parktab = drive.actionBuilder(Robot.Constants.autoShotPose)
                .strafeTo(new Vector2d(-50,20), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-40,100)) // Strafe to parking
                ;

        ArrayList<TrajectoryActionBuilder> trajs = new ArrayList<>();
        trajs.add(tab1); trajs.add(tab); trajs.add(tab2); trajs.add(tab3); trajs.add(parktab); trajs.add(tab4Mysterious);
        ArrayList<TrajectoryActionBuilder> trajs_back = new ArrayList<>();
        trajs_back.add(tab2_back); trajs_back.add(tab3_back); trajs_back.add(tab4_back);
        ArrayList<char[]> queues = new ArrayList<>();
        queues.add(new char[]{'P','P','G'}); queues.add(new char[]{'P','G','P'}); queues.add(new char[]{'G','P','P'});

        if (row == 1){
            //trajs swap tab2 and tab3
            TrajectoryActionBuilder t = trajs.get(2);
            trajs.set(2,trajs.get(3));
            trajs.set(3, t); //doesn't matter in 9 ball but matters in 12 ball
            //trajs back swap tab2 and tab3
            TrajectoryActionBuilder tr = trajs_back.get(0);
            trajs_back.set(0,trajs_back.get(1));
            trajs_back.set(1, tr); //doesn't matter in 9 ball but matters in 12 ball
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
            //trajs back swap tab2 and tab3
            TrajectoryActionBuilder tr = trajs_back.get(0);
            trajs_back.set(0,trajs_back.get(2));
            trajs_back.set(2, tr); //doesn't matter in 9 ball but matters in 12 ball
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
                        robot.updateShooter(shotReqFR.get(), shotReqBL.get(), false, telemetry, true, Robot.Positions.autoShotPose, 0,false,false,false, false, false, false, false, false,false, false, false);
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
        Action firstShot = Robot.shootSequence(shotReqFR, shotReqBL, slowIntake, new char[]{'P','G','P'}, i, 1, robot.opModeState);
        Action secondShot = Robot.shootSequence(shotReqFR, shotReqBL, slowIntake, queues.get(0), i, 2, robot.opModeState);
        Action thirdShot = Robot.shootSequence(shotReqFR, shotReqBL, slowIntake, queues.get(1), i, 3, robot.opModeState);
        Action fourthShot = Robot.shootSequence(shotReqFR, shotReqBL, slowIntake, queues.get(2), i, 4, robot.opModeState);
        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                turnGoal, // Turn 90 degrees
                                // FIRE ROUND 1 (detect obelisk)
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
                            robot.controlIntake(false/*intake.get()*/, reverseIntake.get(), stopIntake.get(), false, intake.get() || slowIntake.get());
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
        finally{
            robot.setPose(drive.localizer.getPose());
            RobotLog.a("Finally");
        }
    }
}
