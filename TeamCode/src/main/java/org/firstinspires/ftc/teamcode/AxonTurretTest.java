package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AxonTurretTest {
    // Encoder for servo position feedback
    private final AnalogInput servoEncoder;
    // Continuous rotation servo
    private final CRServo servo;
    // Run-to-position mode flag
    private boolean rtp;
    // Current power applied to servo
    private double power;
    // Maximum allowed power
    private double maxPower;
    // Direction of servo movement
    private Direction direction;
    // Last measured angle
    private double previousAngle;
    // Accumulated rotation in degrees
    private double totalRotation;
    // Target rotation in degrees
    private double targetRotation;

    // PID controller coefficients and state
    private double kP;
    private double kI;
    private double kD;
    private double integralSum;
    private double lastError;
    private double maxIntegralSum;
    private ElapsedTime pidTimer;

    // Initialization and debug fields
    public double STARTPOS;
    public int ntry = 0;
    public int cliffs = 0;
    public double homeAngle;

    // CRServo stabilization fields
    private boolean startupPhase = true;
    private ElapsedTime startupTimer = new ElapsedTime();
    private static final double STARTUP_DELAY = 0.1; // seconds
    private static final double MIN_POWER = 0.05;    // minimum CRServo power to overcome friction
    private static final double ALPHA = 0.2;         // low-pass filter coefficient
    private static final double MAX_DERIVATIVE = 40;

    // Direction enum for servo
    public enum Direction {
        FORWARD,
        REVERSE
    }

    // region constructors

    // Basic constructor, defaults to FORWARD direction
    public AxonTurretTest(CRServo servo, AnalogInput encoder) {
        rtp = true;
        this.servo = servo;
        servoEncoder = encoder;
        direction = Direction.FORWARD;
        initialize();
    }

    // Initialization logic for servo and encoder
    private void initialize() {
        servo.setPower(0);
        try {
            Thread.sleep(50); // let voltage stabilize
        } catch (InterruptedException ignored) {
        }

        // Read stable starting angle
        double stableAngle = getCurrentAngle();
        STARTPOS = stableAngle;
        previousAngle = stableAngle;
        homeAngle = stableAngle;
        totalRotation = 0;

        // Default PID coefficients
        kP = 0.01;
        kI = 0.0001;
        kD = 0.001;
        integralSum = 0.0;
        lastError = 0.0;
        maxIntegralSum = 100.0;
        pidTimer = new ElapsedTime();
        pidTimer.reset();

        maxPower = 0.3;
        cliffs = 0;
        startupPhase = true;
        startupTimer.reset();
    }
    // endregion

    // Set servo direction
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    // Set power to servo, respecting direction and maxPower
    public void setPower(double power) {
        this.power = Math.max(-maxPower, Math.min(maxPower, power));
        servo.setPower(this.power * (direction == Direction.REVERSE ? -1 : 1));
    }

    public double getPower() { return power; }
    public void setKP(double kP) { this.kP = kP; }
    public void setKI(double kI) { this.kI = kI; resetIntegral(); }
    public void setKD(double kD) { this.kD = kD; }
    public double getKP() { return kP; }
    public double getKI() { return kI; }
    public void changeTargetRotation(double change) { targetRotation += change; }
    public void setTargetRotation(double target) { targetRotation = target; resetPID(); }
    public boolean isAtTarget(double tolerance) { return Math.abs(targetRotation - totalRotation) < tolerance; }
    public void resetPID() { resetIntegral(); lastError = 0; pidTimer.reset(); }
    public void resetIntegral() { integralSum = 0; }

    // current angle
    public double getCurrentAngle() {
        if (servoEncoder == null) return 0;
        double angle = (servoEncoder.getVoltage() / 3.3) * 360.0;
        if (direction == Direction.REVERSE) angle = -angle;
        return angle;
    }

    // Main update loop
    public synchronized void update() {
        // low pass filter
        double rawAngle = getCurrentAngle();
        double currentAngle = ALPHA * rawAngle + (1 - ALPHA) * previousAngle;

        // multiple turns
        double angleDifference = currentAngle - previousAngle;
        if (angleDifference > 180) { angleDifference -= 360; cliffs--; }
        else if (angleDifference < -180) { angleDifference += 360; cliffs++; }

        // update rotation
        totalRotation = currentAngle - homeAngle + cliffs * 360;
        previousAngle = currentAngle;

        if (!rtp) return;

        // PID calcs
        double dt = pidTimer.seconds();
        pidTimer.reset();
        if (dt < 0.001 || dt > 1.0) return;

        double error = targetRotation - totalRotation;
        integralSum += error * dt;
        integralSum = Math.max(-maxIntegralSum, Math.min(maxIntegralSum, integralSum));
        if (Math.abs(error) < 2.0) integralSum *= 0.95;

        double derivative = (error - lastError) / dt;
        derivative = Math.max(-MAX_DERIVATIVE, Math.min(MAX_DERIVATIVE, derivative));
        lastError = error;

        double output = kP * error + kI * integralSum + kD * derivative;
        output = Math.max(-maxPower, Math.min(maxPower, output));

        if (startupPhase) { // ignore small movements at start?
            if (startupTimer.seconds() < STARTUP_DELAY) {
                output = 0;
            } else {
                startupPhase = false;
            }
        }

        if (Math.abs(output) > 0 && Math.abs(output) < MIN_POWER) { // min power threshhold
            output = Math.signum(output) * MIN_POWER;
        }

        setPower(output);
    }

    // Telemetry logging
    @SuppressLint("DefaultLocale")
    public String log() {
        return String.format(
                "Current Volts: %.3f\n" +
                        "Current Angle: %.2f\n" +
                        "Total Rotation: %.2f\n" +
                        "Target Rotation: %.2f\n" +
                        "Current Power: %.3f\n" +
                        "PID Values: P=%.3f I=%.3f D=%.3f\n" +
                        "PID Terms: Error=%.2f Integral=%.2f",
                servoEncoder.getVoltage(),
                getCurrentAngle(),
                totalRotation,
                targetRotation,
                power,
                kP, kI, kD,
                targetRotation - totalRotation,
                integralSum
        );
    }

    // TeleOp test class
    @TeleOp(name = "AxonTurretTest")
    public static class CRAxonTest extends LinearOpMode {

        @Override
        public void runOpMode() throws InterruptedException {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            CRServo crservo = hardwareMap.crservo.get("servorot");
            AnalogInput encoder = hardwareMap.get(AnalogInput.class, "axonAnalog");
            AxonTurretTest servo = new AxonTurretTest(crservo, encoder);

            boolean lastDpadUp = false;
            boolean lastDpadDown = false;
            boolean lastX = false;
            boolean lastY = false;
            boolean lastA = false;
            boolean lastRightBumper = false;
            boolean lastLeftBumper = false;
            boolean lastB = false;

            waitForStart();

            while (!isStopRequested()) {
                servo.update();

                // Button handling
                boolean currDpadUp = gamepad1.dpad_up;
                boolean currDpadDown = gamepad1.dpad_down;
                boolean currX = gamepad1.x;
                boolean currY = gamepad1.y;
                boolean currA = gamepad1.a;
                boolean currRightBumper = gamepad1.right_bumper;
                boolean currLeftBumper = gamepad1.left_bumper;
                boolean currB = gamepad1.b;

                if (currDpadUp && !lastDpadUp) servo.changeTargetRotation(90);
                if (currDpadDown && !lastDpadDown) servo.changeTargetRotation(-90);
                if (currX && !lastX) servo.setTargetRotation(0);
                if (currY && !lastY) servo.setKP(servo.getKP() + 0.001);
                if (currA && !lastA) servo.setKP(Math.max(0, servo.getKP() - 0.001));
                if (currRightBumper && !lastRightBumper) servo.setKI(servo.getKI() + 0.0001);
                if (currLeftBumper && !lastLeftBumper) servo.setKI(Math.max(0, servo.getKI() - 0.0001));
                if (currB && !lastB) {
                    servo.setKP(0.015);
                    servo.setKI(0.0005);
                    servo.setKD(0.0025);
                    servo.resetPID();
                }

                lastDpadUp = currDpadUp;
                lastDpadDown = currDpadDown;
                lastX = currX;
                lastY = currY;
                lastA = currA;
                lastRightBumper = currRightBumper;
                lastLeftBumper = currLeftBumper;
                lastB = currB;

                telemetry.addData("Starting angle", servo.STARTPOS);
                telemetry.addLine(servo.log());
                telemetry.addData("NTRY", servo.ntry);
                telemetry.addData("Voltage", encoder.getVoltage());
                telemetry.update();
            }
        }
    }
}
