package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//ONLY A TEST
@TeleOp
public class TeleopSG extends OpMode {
    RobotSG robot;
    public void init(){
        robot = RobotSG.getInstance(hardwareMap, new Pose2d(0,0,0));
        robot.initTeleop(telemetry);
        telemetry.update();
    }
    public void loop(){
        robot.updateLocalizer(telemetry);
        robot.controlIntake(gamepad1.a, gamepad1.b, gamepad1.x);
        robot.updateShooter(gamepad1.right_bumper, telemetry);
        robot.driveFieldCentric(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        telemetry.update();
    }
}
