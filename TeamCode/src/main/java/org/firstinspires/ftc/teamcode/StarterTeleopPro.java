package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class StarterTeleopPro extends OpMode {
    StarterRobotPro robot;
    public void init(){
        robot = new StarterRobotPro(hardwareMap);
        robot.initTeleop(telemetry);
        telemetry.update();
    }
    public void loop(){
        robot.updateLocalizer(telemetry);
        robot.updateShooter(gamepad1.right_bumper, telemetry);
        robot.driveFieldCentric(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, telemetry);
        telemetry.update();
    }
}
