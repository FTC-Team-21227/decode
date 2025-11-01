package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Teleop extends OpMode {
    Robot robot;
    boolean intake = false; //false = stop, true = intake
    boolean setPose = false;
    boolean human = false;
//    FtcDashboard dashboard = FtcDashboard.getInstance();
//    Telemetry telemetry = dashboard.getTelemetry();
    TelemetryPacket packet = new TelemetryPacket();
    public void init(){
        robot = new Robot(hardwareMap, new Pose2d(0,0, Math.PI), Robot.Color.RED); //start facing the goals, RED poses
        robot.initTeleop(telemetry);
        telemetry.update();
    }
    public void loop(){
        //toggles: LB, RB, x, y
        if (gamepad1.leftBumperWasPressed()) intake = !intake;
        if (gamepad1.xWasPressed()) setPose = !setPose;
        if (gamepad1.yWasPressed()) human = !human;
        //final: back = relocalize
        robot.updateLocalizer(gamepad1.backWasPressed(), telemetry);
        //final version will be: LB = intake toggle, b = unpower
        robot.controlIntake(intake, gamepad1.b, !intake);
        //final version will be: RT = front feeder, LT = back feeder, RB = alternating feeder, x = toggle setPose => shooter lock/manual control, rightstick up/down = flywheel scale, dpad up/down= hood, dpad left/right = turret, y = human feed toggle
        robot.updateShooter(gamepad1.right_trigger > 0.1, gamepad1.left_trigger > 0.1, gamepad1.right_bumper, telemetry, setPose, Robot.Constants.autoShotPose, gamepad1.right_stick_y, gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_left, gamepad1.dpad_right, human);
        //robot.controlPark: toggle start = up/down. Everything unpowered when up.
        robot.driveFieldCentric(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        robot.drive2.drawPose(packet);
        telemetry.update();
    }
}
