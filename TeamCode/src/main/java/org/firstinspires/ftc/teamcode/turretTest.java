package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class turretTest extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();
    Turret turret;

    @Override
    public void runOpMode() throws InterruptedException {
        turret = new Turret(hardwareMap);
        waitForStart();
        double turretPos = 0.87;
        while (opModeIsActive()) {
            // Run flywheel based on calculated velocity
//            flywheelVelocity = calcShooterVel(telemetry, distance, heightDiff);
//            setFlyWheelSpeed(flywheelVelocity, gamepad1.a);
            if (gamepad1.a){
                turret.turnToRobotAngle(0);
            }
            if (gamepad1.b){
                turret.turnToRobotAngle(Math.PI);
            }
            if (gamepad1.dpad_left){
                turret.turnToRobotAngle(14*Math.PI/180);
            }

            if (gamepad1.yWasPressed()){
                turret.turret.setPosition(0);
                sleep(5000);
                for(double p = 0; p <= 1; p += 0.01) {
                    turret.turret.setPosition(p);
                    sleep(100);
                }
                for(double p = 1; p >= 0; p -= 0.01) {
                    turret.turret.setPosition(p);
                    sleep(100);
                }
            }
            if (gamepad1.xWasPressed()){
                turret.turnToRobotAngle(0);
                sleep(5000);
                for(double a = 0; a <= Math.PI; a += 0.01*Math.PI) {
                    turret.turnToRobotAngle(a);
                    sleep(100);
                }
                for(double a = 1; a >= -Math.PI; a -= 0.01*Math.PI) {
                    turret.turnToRobotAngle(a);
                    sleep(100);
                }
            }
            if (gamepad1.dpad_up){
                turretPos += 0.001;
                turret.turret.setPosition(turretPos);
            }
            if (gamepad1.dpad_down){
                turretPos -= 0.001;
                turret.turret.setPosition(turretPos);
            }

            // Telemetry lines
            telemetry.addData("turret angle", Math.toDegrees(turret.getTurretRobotAngle()));
            telemetry.addData("turret pos", turret.turret.getPosition());
            telemetry.update();

        }
    }

};