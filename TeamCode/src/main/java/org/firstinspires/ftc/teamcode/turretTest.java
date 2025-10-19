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
    Turret turret;//    private DigitalChannel LED_DigitalChannel;

    @Override
    public void runOpMode() throws InterruptedException {
 //        initialization();
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