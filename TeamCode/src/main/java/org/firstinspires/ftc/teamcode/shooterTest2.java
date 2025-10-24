package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Shooter Test 2")
public class shooterTest2 extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();
    ShooterRobot robot;
    Intake intake;

    // Distance and height difference from robot to goal
    public static double distance;
    public static double heightDiff;

    double flywheelVelocity;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new ShooterRobot(hardwareMap, new Pose2d(0, 0, Math.PI), ShooterRobot.Color.RED); // Facing forward at (0,0)
        intake= new Intake(hardwareMap);
        robot.initTeleop(telemetry);

        waitForStart();

        double turretPos = 1;
        double hoodPos = 0;

        while (opModeIsActive()) {
            intake.intake(); // Start spinning intake
            if (gamepad1.a){ // Turret face forwards
                robot.turret.turnToRobotAngle(0);
            }
            if (gamepad1.b){ // Turret face backwards
                robot.turret.turnToRobotAngle(Math.PI);
            }
            /*
            if (gamepad1.x){ // Hood completely down
                robot.hood.turnToAngle(0);
            }
            if (gamepad1.y){ // Hood overhead angle
                robot.hood.turnToAngle(45);
            }
            */
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
            // Will start spinning flywheel, move turret and hood, but will not go through full shooting process until START is pressed
            robot.updateShooter(gamepad1.start, gamepad1.dpad_up, gamepad1.dpad_down, telemetry);

            // Telemetry lines
            telemetry.addData("turret angle", Math.toDegrees(robot.turret.getTurretRobotAngle()));
            telemetry.addData("turret pos", robot.turret.turret.getPosition());
            telemetry.addData("Distance", distance);
            telemetry.addData("Height Diff", heightDiff);
            telemetry.addData("Flywheel Target Vel", flywheelVelocity);
            telemetry.update();

        }
    }

};