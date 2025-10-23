package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Shooter Test")
public class shooterTest extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();
    ShooterRobot robot;

    // Editable in dashboard, distance and height difference from robot to goal
    public static double distance;
    public static double heightDiff;

    double flywheelVelocity;
    double HoodPosition;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new ShooterRobot(hardwareMap, new Pose2d(0, 0, /*Math.PI / 2*/ 0), ShooterRobot.Color.RED);
//        initialization();
        robot.initTeleop(telemetry);
        waitForStart();
        double turretPos = 1;
        double hoodPos = 0;
        while (opModeIsActive()) {
            // Run flywheel based on calculated velocity
//            flywheelVelocity = calcShooterVel(telemetry, distance, heightDiff);
//            setFlyWheelSpeed(flywheelVelocity, gamepad1.a);
            if (gamepad1.a){ // Turret face forwards
                robot.turret.turnToRobotAngle(0);
            }
            if (gamepad1.b){ // Turret face backwards
                robot.turret.turnToRobotAngle(Math.PI);
            }
            if (gamepad1.dpad_left){ // Turret face 14 degrees
                robot.turret.turnToRobotAngle(14 * Math.PI / 180);
            }
            if (gamepad1.x){ // Hood angle
                robot.hood.turnToAngle(0);
            }
            if (gamepad1.y){ // Hood overhead angle
                robot.hood.turnToAngle(45);
            }
            if (gamepad1.dpad_up){ // Turret turn left
                turretPos += 0.001;
                robot.turret.turret.setPosition(turretPos);
            }
            if (gamepad1.dpad_down){ // Turret turn right
                turretPos -= 0.001;
                robot.turret.turret.setPosition(turretPos);
            }
            if (gamepad1.left_bumper){ // Move hood down
                hoodPos += 0.001;
                robot.hood.HOOD.setPosition(hoodPos);
            }
            if (gamepad1.right_bumper){ // Move hood up
                hoodPos -= 0.001;
                robot.hood.HOOD.setPosition(hoodPos);
            }
            robot.updateLocalizer(telemetry);
            robot.updateShooter(gamepad1.start, gamepad1.dpad_up, gamepad1.dpad_down, telemetry);
//            robot.flywheel.spinTo(20);

            // Telemetry lines
            telemetry.addData("turret angle", Math.toDegrees(robot.turret.getTurretRobotAngle()));
            telemetry.addData("turret pos", robot.turret.turret.getPosition());
            telemetry.addData("Distance", distance);
            telemetry.addData("Height Diff", heightDiff);
            telemetry.addData("Flywheel Target Vel", flywheelVelocity);
//            telemetry.addData("Flywheel Cur Vel", ((DcMotorEx) FlyWheelMotor).getVelocity());
            telemetry.update();

        }
    }

};