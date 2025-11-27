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


@Autonomous(name = "Blue_Far_9_Human")
public class Auto_Far_Blue_Goal_6_Human extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.Color color = Robot.Color.BLUE;

        MecanumDrive.PARAMS.maxWheelVel = 50;
        MecanumDrive.PARAMS.minProfileAccel = -30;
        MecanumDrive.PARAMS.maxProfileAccel = 50;
        Robot.Constants.flywheelPower = 2.3;
//        Pose2d initialPose = new Pose2d(-55, 46, Math.toRadians(-55));
        Pose2d initialPose = new Pose2d(62, -14, Math.toRadians(180));
        Robot robot = Robot.startInstance(initialPose, color);
        robot.initAuto(hardwareMap, telemetry, Robot.OpModeState.AUTO_FAR);
        telemetry.update();
        MecanumDrive drive = robot.drive2;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose) //first specimen
                .strafeTo(new Vector2d(56,-12))
                .turnTo(Math.toRadians(-120));
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(56,-12,Math.toRadians(-120)))
                .strafeToLinearHeading(new Vector2d(60,-55),Math.toRadians(-90))
                .strafeTo(new Vector2d(60,-60))
                ;
        TrajectoryActionBuilder tab2_back = drive.actionBuilder(new Pose2d(/*37*/ 60,-60,Math.toRadians(-90))) //first specimen
                .strafeTo(new Vector2d(30,-56))
                .strafeToLinearHeading(new Vector2d(56,-12),Math.toRadians(-120))
                ;
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(56,-12,Math.toRadians(-120)))
                .strafeToLinearHeading(new Vector2d(60,-55),Math.toRadians(-90))
                .strafeTo(new Vector2d(60,-60))
                ;
        TrajectoryActionBuilder tab3_back = drive.actionBuilder(new Pose2d(/*37*/ 60,-60,Math.toRadians(-90))) //first specimen
                .strafeTo(new Vector2d(30,-56))
                .strafeToLinearHeading(new Vector2d(56,-12),Math.toRadians(-120))
                ;
//        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(56,12,Math.toRadians(90)))
//                .strafeTo(new Vector2d(36,15))
//                .strafeTo(new Vector2d(36,45), new TranslationalVelConstraint(15))
//                .strafeTo(new Vector2d(-12,15))
//                ;
        TrajectoryActionBuilder tabp = drive.actionBuilder(new Pose2d(56,-12,Math.toRadians(-120)))
                .strafeTo(new Vector2d(56,-32)) // Strafe to parking
                ;

        Action parkTrajectory = tabp.build();

        Action firstTrajectory = tab1.build();
        Action secondTrajectory = tab2.build();
        Action secondTrajectory_back = tab2_back.build();
        Action thirdTrajectory = tab3.build();
        Action thirdTrajectory_back = tab3_back.build();

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
        try {
            Actions.runBlocking(
                    new ParallelAction(
                            new SequentialAction(
                                    new InstantAction(() -> detectOb.set(true)),
                                    new SleepAction(0.5),
                                    new InstantAction(() -> detectOb.set(false)),
                                    new InstantAction(() -> con.set(false))
                            ),
                            telemetryPacket -> {
                                robot.controlIntake(intake.get(), reverseIntake.get(), stopIntake.get(), false, slowIntake.get(), false);
                                robot.updateShooter(shotReqFR.get(), shotReqBL.get(), false, telemetry, true, Robot.Positions.autoShotPose_Far, 0, false,false, false, false, false, false, false, false, false, false, false);
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
            Action firstShot = Robot.shootSequence(shotReqFR, shotReqBL, slowIntake, new char[]{'P', 'G', 'P'}, i, 1, Robot.OpModeState.AUTO_FAR, Robot.Color.BLUE);
            Action secondShot = Robot.shootSequence(shotReqFR, shotReqBL, slowIntake, new char[]{'G', 'P', 'P'}, i, 2, Robot.OpModeState.AUTO_FAR, Robot.Color.BLUE);
            Action thirdShot = Robot.shootSequence(shotReqFR, shotReqBL, slowIntake, new char[]{'P', 'G', 'P'}, i, 3, Robot.OpModeState.AUTO_FAR, Robot.Color.BLUE);
            Actions.runBlocking(
                    new ParallelAction(
                            new SequentialAction(
                                    firstTrajectory,
                                    firstShot,
                                    // FIRE ROUND 1 (detect obelisk)
                                    new InstantAction(() -> slowIntake.set(true)),
                                    // COLLECT ROUND 2 BALLS
                                    secondTrajectory,
                                    new ParallelAction(
                                            secondTrajectory_back,
                                            new SequentialAction(
                                                    new InstantAction(() -> slowIntake.set(false)),
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
                                                    new InstantAction(() -> slowIntake.set(false)),
                                                    new SleepAction(0.5),
                                                    new InstantAction(() -> reverseIntake.set(true)),
                                                    new SleepAction(Robot.Constants.outtakePulseTime),
                                                    new InstantAction(() -> reverseIntake.set(false)),
                                                    new InstantAction(() -> stopIntake.set(true))
                                            )
                                    ),

                                    // FIRE ROUND 3
                                    thirdShot,
                                    parkTrajectory
                                    //END
                            ),
                            telemetryPacket -> {
                                robot.controlIntake(intake.get(), reverseIntake.get(), stopIntake.get(), false, slowIntake.get(), false);
                                robot.updateShooter(shotReqFR.get(), shotReqBL.get(), false, telemetry, true, Robot.Positions.autoShotPose_Far, 0, false,false, false, false, false, false, false, false, false, false, false);
//                            id.set(robot.camera.detectObelisk(telemetry, detectOb.get()));
                                telemetry.addData("Obelisk Detected", id.get());
                                int[] order = Robot.Order;
                                telemetryPacket.put("order", order[0] + ", " + order[1] + ", " + order[2]); // Feeder order
                                telemetry.update();
                                return true;
                            }

                    )
            );
        }
        finally{
            robot.setPose(drive.localizer.getPose());
            MecanumDrive.PARAMS.maxWheelVel = 80;
            MecanumDrive.PARAMS.minProfileAccel = -40;
            MecanumDrive.PARAMS.maxProfileAccel = 80;
            RobotLog.a("Finally");
            }
    }
}
