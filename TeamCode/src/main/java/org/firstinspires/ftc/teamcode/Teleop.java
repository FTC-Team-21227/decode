package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class Teleop extends OpMode {
    Robot robot;
//    FtcDashboard dashboard = FtcDashboard.getInstance();
//    Telemetry telemetry = dashboard.getTelemetry();
    public void init(){
        robot = new Robot(hardwareMap, new Pose2d(0,0, Math.PI), Robot.Color.RED); //start facing the goals, RED poses
        robot.initTeleop(telemetry);
        telemetry.update();
    }
    public void loop(){
        robot.updateLocalizer(telemetry);
        robot.controlIntake(gamepad1.a, gamepad1.b, gamepad1.x);
        robot.updateShooter(gamepad1.left_bumper, gamepad1.right_bumper, telemetry);
        robot.driveFieldCentric(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        telemetry.update();
    }
}
