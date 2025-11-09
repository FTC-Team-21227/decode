package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.DriveConstants;

@Autonomous(name = "MeepMeepAuton", group = "Autonomous")
public class MeepMeepAuton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // starting pose = meep meep start pose
        Pose2d startPose = new Pose2d(0, 0, 0);

        // init drive
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose, null);

        // build same trajectory as your MeepMeep path
        TrajectoryActionBuilder traj = drive.actionBuilder(startPose)
                .lineToX(30)
                .turn(Math.toRadians(90))
                .lineToY(30)
                .turn(Math.toRadians(90))
                .lineToX(0)
                .turn(Math.toRadians(90))
                .lineToY(0)
                .turn(Math.toRadians(90));

        Action trajectoryAction = traj.build();

        // Wait for the game to start
        waitForStart();

        if (isStopRequested()) return;

        // Follow the trajectory
        Actions.runBlocking(trajectoryAction);
    }
}
