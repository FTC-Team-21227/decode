package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Shooter Test No Turret")
public class shooterTest_NoTurret extends LinearOpMode {

    Flywheel flywheel;
    Servo hood;
    // Editable in dashboard, distance and height difference from robot to goal
    public static double distance;
    public static double heightDiff;

    double flywheelVelocity;
    double HoodPosition;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry (FtcDashboard.getInstance().getTelemetry());
        flywheel = new Flywheel(hardwareMap);
        hood = hardwareMap.get(Servo.class, "hood");
        //        initialization();
        waitForStart();
        double hoodPos = 0.5;
        flywheelVelocity = 0;
        while (opModeIsActive()) {
            if (gamepad1.left_bumper){ // Move hood down
                hoodPos += 0.001;
                hood.setPosition(hoodPos);
            }
            if (gamepad1.right_bumper){ // Move hood up
                hoodPos -= 0.001;
                hood.setPosition(hoodPos);
            }
            if (gamepad1.dpad_up){
                flywheelVelocity += 20;
                flywheel.spinTo(flywheelVelocity);
            }
            if (gamepad1.dpad_down){
                flywheelVelocity -= 20;
                flywheel.spinTo(flywheelVelocity);
            }
            telemetry.addData("Flywheel Target Vel", flywheelVelocity);
            telemetry.addData("Flywheel Current Vel", flywheel.getVel());
            telemetry.addData("hood pos", hood.getPosition());
            telemetry.update();

        }
    }

};