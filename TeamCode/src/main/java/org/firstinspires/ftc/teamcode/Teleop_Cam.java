package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Teleop_Cam extends OpMode {
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
        robot.camera.detectObelisk(telemetry,true);
        telemetry.update();
    }
}
