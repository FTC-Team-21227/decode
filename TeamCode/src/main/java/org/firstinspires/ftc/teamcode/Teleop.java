package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Teleop extends OpMode {
    Robot robot;
//    FtcDashboard dashboard = FtcDashboard.getInstance();
//    Telemetry telemetry = dashboard.getTelemetry();
    TelemetryPacket packet = new TelemetryPacket();
    public void init(){
        robot = new Robot(hardwareMap, new Pose2d(0,0, Math.PI), Robot.Color.RED); //start facing the goals, RED poses
        robot.initTeleop(telemetry);
        telemetry.update();
    }
    public void loop(){
        //final: back = relocalize
        robot.updateLocalizer(telemetry);
        //final version will be: LT = intake toggle, b = unpower
        robot.controlIntake(gamepad1.a, gamepad1.b, gamepad1.x);
        //final version will be: RT = front feeder, LT = back feeder, RB = alternating feeder, x = toggle setPose => shooter lock, a = toggle manual control, rightstick up/down = flywheel scale, dpad up/down= hood, dpad left/right = turret, y = human feed toggle
        robot.updateShooter(gamepad1.left_bumper, gamepad1.right_bumper, telemetry, false, null, gamepad1.right_trigger > 0.1);
        //robot.controlPark: toggle start = up/down. Everything unpowered when up.
        robot.driveFieldCentric(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        robot.drive2.drawPose(packet);
        telemetry.update();
    }
}
