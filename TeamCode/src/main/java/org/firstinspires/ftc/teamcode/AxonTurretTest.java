package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import android.annotation.SuppressLint;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Axon MINI MK2 Example")
public class ServoOpMode extends LinearOpMode {
    private CRServo servo;
    private AnalogInput encoder;

    private double previousAngle = 0; // angle before rotation
    private int cliffs = 0; // full revolutions
    private double homeAngle = 0; // original angle
    private double totalAxonRotation = 0;

    private double targetRotation = 90; // target angle of rotation
    private double maxPower = 0.6;
    private final double tolerance = 10.0; // degrees

    private final double axonToTurretRatio = 1.0; // turretAngle / axonAngle

    public enum Direction {
        FORWARD,
        REVERSE
    }
    private Direction direction = Direction.FORWARD;

    @Override
    public void runOpMode() throws InterruptedException {
        // Map hardware here
        servo = hardwareMap.crservo.get("servorot");
        encoder = hardwareMap.get(AnalogInput.class, "axonAnalog");

        previousAngle = getCurrentAngle(); // angle before rotation
        homeAngle = previousAngle; // save angle before rotation

        waitForStart();

        while (opModeIsActive()) {
            update();

            telemetry.addData("Axon Rotation", totalAxonRotation);
            telemetry.addData("Turret Rotation", totalAxonRotation * axonToTurretRatio);
            telemetry.addData("Target Rotation", targetRotation);
            telemetry.addData("At Target?", isAtTarget());
            telemetry.update();
        }
    }

    // set servo direction
    public void setDirection(Direction dir) {
        this.direction = dir;
    }

    // set max power
    public void setMaxPower(double power) {
        maxPower = power;
    }

    // set target rotation
    public void setTargetRotation(double target) {
        targetRotation = target;
    }

    // get current analog angle
    public double getCurrentAngle() {
        return (encoder.getVoltage() / 3.3) * (direction == Direction.REVERSE ? -360 : 360);
    }

    // update total rotation for multi-turn tracking
    public void update() {
        double currentAngle = getCurrentAngle(); // angle of CRServo from AnalogInput
//        double delta = currentAngle - previousAngle;
//
//        // wraparound
//        if (delta > 180) delta -= 360;
//        else if (delta < -180) delta += 360;
//
////        cliffs += (delta > 0 && currentAngle < previousAngle) ? 1 : (delta < 0 && currentAngle > previousAngle ? -1 : 0);
//        if (delta > 180) {
//            delta -= 360;
//            cliffs--;
//        } else if (delta < -180) {
//            delta += 360;
//            cliffs++;
//        }
        totalAxonRotation = (currentAngle - homeAngle + cliffs * 360) * axonToTurretRatio;
        previousAngle = currentAngle;

        double error = targetRotation - totalAxonRotation;

        if (Math.abs(error) < tolerance) {
            servo.setPower(0); // Stop when within tolerance
        } else if (error > 0) {
            servo.setPower(maxPower); // Move forward
        } else {
            servo.setPower(-maxPower); // Move backward
        }
    }

    // check if at target
    public boolean isAtTarget() {
        return Math.abs(targetRotation - totalAxonRotation) < tolerance;
    }

    // total rotation
    public double getAxonRotation() {
        return totalAxonRotation;
    }

    public double getTurretRotation() {
        return totalAxonRotation * axonToTurretRatio;
    }
}