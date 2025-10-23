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


@Autonomous(name = "Auton Red Goal")
public class Auto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-55, 46, Math.toRadians(-55)); // In front of goal
        MecanumDrive robot = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder tab1 = robot.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-12,46-20*Math.tan(Math.toRadians(55))))
                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(-12,45))
                .strafeTo(new Vector2d(-12,15))
                ;
        TrajectoryActionBuilder tab2 = robot.actionBuilder(new Pose2d(-12,15,Math.toRadians(90)))
                .strafeTo(new Vector2d(12,15))
                .strafeTo(new Vector2d(12,45))
                .strafeTo(new Vector2d(-12,15))
                ;

        Action firstTrajectory = tab1.build();
        Action secondTrajectory = tab2.build();
//        Action p2p = robot.drive2.p2pAction();
        waitForStart();

        AtomicBoolean shotReq = new AtomicBoolean(false);
        Actions.runBlocking(
//                new ParallelAction(
                        new SequentialAction(
//                            new SleepAction(5), // Wait for flywheel to charge up
                            // Give 3 seconds to shoot the first set of 3 balls
                            new InstantAction(() -> shotReq.set(true)),
//                            new SleepAction(3),
                            new InstantAction(() -> shotReq.set(false)),
                            firstTrajectory, // Gobble up row 1
                            // Give 3 seconds to shoot the second set of 3 balls
                            new InstantAction(() -> shotReq.set(true)),
                            new SleepAction(3),
                            new InstantAction(() -> shotReq.set(false)),
                            secondTrajectory, // Gobble up row 2
                            // Give 3 seconds to shoot the third set of 3 balls
                            new InstantAction(() -> shotReq.set(true)),
                            new SleepAction(3),
                            new InstantAction(() -> shotReq.set(false))
                        )
                /*
                        telemetryPacket -> {
                            robot.controlIntake(false,false,true);
                            robot.updateShooter(shotReq.get(),telemetry);
                            return true;
                        }

                )
                 */
        );
    }
}
