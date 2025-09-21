package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "Auto")
public class Auto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(10.5, -63.3, Math.toRadians(90));
        Robot robot = new Robot(hardwareMap, initialPose, Robot.Color.RED);

        TrajectoryActionBuilder tab1 = robot.drive.actionBuilder(initialPose) //first specimen
//                .setTangent(Math.toRadians(90))
                .waitSeconds(0.3)
//                .splineToConstantHeading(new Vector2d(pose_X,firstSpecDistance),Math.toRadians(90));
                .strafeTo(new Vector2d(10.5,10));

        Action firstTrajectory = tab1.build();

        waitForStart();

        Actions.runBlocking(
//                new SequentialAction(
                //first specimen
                new ParallelAction(
                        firstTrajectory,
                        //shotReq = false
                        //delayTillShot = 1
                        //second,
                        //shotReq = true

                        telemetryPacket -> {
                            robot.updateShooter(true,telemetry);
                            return true;
                        }
                )
//                )
        );
    }
}
