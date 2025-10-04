package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class StarterTeleop extends OpMode {
    StarterRobot robot;
    public void init(){
        robot = new StarterRobot(hardwareMap);
        robot.initTeleop(telemetry);
        telemetry.update();
    }
    public void loop(){
        robot.updateLocalizer(telemetry);
        robot.updateShooter(gamepad1.right_bumper, gamepad1.dpad_up, gamepad1.dpad_down, telemetry);
        robot.driveFieldCentric(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, telemetry);
        telemetry.update();
    }
}
