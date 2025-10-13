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


@Autonomous(name = "Auto_Red_Goal")
public class Auto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-55, 46, Math.toRadians(-55));
        Robot robot = new Robot(hardwareMap, initialPose, Robot.Color.RED);

        TrajectoryActionBuilder tab1 = robot.drive.actionBuilder(initialPose) //first specimen
                .splineTo(new Vector2d(-55+20,46-20*Math.tan(Math.toRadians(55))),Math.toRadians(-55))
                .splineTo(new Vector2d(-12,52),Math.toRadians(90))
                ;
        TrajectoryActionBuilder tab2 = robot.drive.actionBuilder(initialPose) //first specimen
                .setTangent(Math.toRadians(45))
                .splineTo(new Vector2d(12,52),Math.toRadians(90))
                .strafeTo(new Vector2d(-12,15))
                ;

        Action firstTrajectory = tab1.build();
        Action secondTrajectory = tab2.build();
        waitForStart();

        AtomicBoolean shotReq = new AtomicBoolean(false);
        Actions.runBlocking(
//                new SequentialAction(
                //first specimen
                new ParallelAction(
                        new SequentialAction(
                            new SleepAction(5), //wait for flywheel to charge up
                            //give 3 seconds to shoot the first set of 3 balls
                            new InstantAction(() -> shotReq.set(true)),
                            new SleepAction(3),
                            new InstantAction(() -> shotReq.set(false)),
                            firstTrajectory, //gobble up row 1
                            //give 3 seconds to shoot the second set of 3 balls
                            new InstantAction(() -> shotReq.set(true)),
                            new SleepAction(3),
                            new InstantAction(() -> shotReq.set(false)),
                            secondTrajectory, //gobble up row 2
                            //give 3 seconds to shoot the third set of 3 balls
                            new InstantAction(() -> shotReq.set(true)),
                            new SleepAction(3),
                            new InstantAction(() -> shotReq.set(false))
                        ),
                        telemetryPacket -> {
                            robot.updateShooter(shotReq.get(),telemetry);
                            return true;
                        }
                )
//                )
        );
    }
}
