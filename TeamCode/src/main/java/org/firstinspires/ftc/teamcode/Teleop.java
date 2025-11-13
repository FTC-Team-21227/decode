package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
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
    boolean slow = false;
//    FtcDashboard dashboard = FtcDashboard.getInstance();
//    Telemetry telemetry = dashboard.getTelemetry();
    TelemetryPacket packet = new TelemetryPacket();
    public void init(){
        robot = Robot.getInstance(new Pose2d(0,0, Math.PI), Robot.Color.RED); //start facing the goals, RED poses
        robot.initTeleop(hardwareMap, telemetry);
//        robot.turret.turnToRobotAngle(0);
//        robot.hood.turnToAngle(Math.toRadians(45));
        telemetry.update();
    }
    public void start(){
        robot.startTeleop();
    }
//    double turretPos = 0.5;
    public void loop(){
//        if (gamepad1.dpad_up){
//            turretPos += 0.001;
//            robot.turret.turret.setPosition(turretPos);
//        }
//        if (gamepad1.dpad_down){
//            turretPos -= 0.001;
//            robot.turret.turret.setPosition(turretPos);
//        }
        //toggles: LB, x, y
        if (gamepad1.leftBumperWasPressed()) intake = !intake;
        if (gamepad1.xWasPressed()) setPose = !setPose;
        if (gamepad1.yWasPressed()) human = !human;
        if (gamepad2.leftStickButtonWasPressed()) slow = !slow;
//        robot.updateVoltage(telemetry);
        //final: back = relocalize
        robot.updateLocalizer(gamepad1.backWasPressed(), telemetry);
        //final version will be: LB = intake toggle, b = reverse, a = brief reverse, inslow needed?
        robot.controlIntake(intake, gamepad1.b, !intake, gamepad1.aWasPressed(), false);
        robot.setGoalTarget();
        //final version will be: RT = front feeder, LT = back feeder, RB = alternating feeder, x = toggle setPose => shooter lock/manual control, rightstick up/down = flywheel scale, dpad up/down= hood, dpad left/right = turret, y = human feed toggle, start = power flywheel off, gamepad2 bumpers and triggers = feeders manual control
        robot.updateShooter(gamepad1.right_trigger > 0.1, gamepad1.left_trigger > 0.1, gamepad1.right_bumper, telemetry, setPose, Robot.Positions.teleShotPose, gamepad1.right_stick_y + gamepad2.right_stick_y, gamepad1.dpad_up || gamepad2.dpad_up, gamepad1.dpad_down || gamepad2.dpad_down, gamepad1.dpad_left || gamepad2.dpad_left, gamepad1.dpad_right || gamepad2.dpad_right, human);
        //final: toggle left stick button = slow mode.
        robot.driveFieldCentric(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, slow);
        robot.drive2.drawPose(packet);
//        telemetry.addLine(""+robot.turret.turret.getPosition());
//        telemetry.addLine(""+robot.drive2.localizer.getPose().position.x + ", "+robot.drive2.localizer.getPose().position.y + ", "+robot.drive2.localizer.getPose().heading.toDouble());
//        telemetry.addLine(""+robot.hood.HOOD.getPosition());
//        telemetry.addLine(""+robot.flywheel.FLYWHEEL.getMotor().getDirection());
//        telemetry.addLine(""+robot.feeder.BL_FEEDER.getPosition());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        telemetry.update();
    }
    public void stop(){
        robot.setPose(robot.drive2.localizer.getPose());
    }
}
