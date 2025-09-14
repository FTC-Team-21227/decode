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
//        robot.updateLocalizer();
        robot.updateShooter(gamepad1.right_bumper, telemetry);
        telemetry.update();
    }
}
