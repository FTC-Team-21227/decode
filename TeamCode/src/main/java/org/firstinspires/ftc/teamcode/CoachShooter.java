package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Coach Shooter Tele")
public class CoachShooter extends LinearOpMode {

    private Servo Guard_Angle;
    private CRServo Shooter_Angle;
    private DcMotor Fly_Wheel;
    private DigitalChannel LED_DigitalChannel;

    double Guard_Position;
    int Wheel_Speed;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double Ready;

        Guard_Angle = hardwareMap.get(Servo.class, "Guard_Angle");
        Shooter_Angle = hardwareMap.get(CRServo.class, "Shooter_Angle");
        Fly_Wheel = hardwareMap.get(DcMotor.class, "Fly_Wheel");
        LED_DigitalChannel = hardwareMap.get(DigitalChannel.class, "LED");

        Initialization();
        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                if (gamepad1.yWasPressed()) {
                    Guard_Position = Math.min(Guard_Position + 0.05, 1);
                    Guard_Angle.setPosition(Guard_Position);
                }
                if (gamepad1.aWasPressed()) {
                    Guard_Position = Math.max(Guard_Position - 0.05, 0);
                    Guard_Angle.setPosition(Guard_Position);
                }
                if (Math.abs(gamepad1.right_stick_x) > 0.1) {
                    Shooter_Angle.setPower(gamepad1.right_stick_x);
                } else {
                    Shooter_Angle.setPower(0);
                }
                if (gamepad1.dpadUpWasPressed()) {
                    Wheel_Speed = Wheel_Speed + 100;
                    Set_FLY_Wheel_Speed();
                }
                if (gamepad1.dpadDownWasPressed()) {
                    Wheel_Speed = Wheel_Speed - 100;
                    Set_FLY_Wheel_Speed();
                }
                if (gamepad1.dpadLeftWasPressed()) {
                    Wheel_Speed = 0;
                    Set_FLY_Wheel_Speed();
                }
                if (Math.abs(Wheel_Speed - ((DcMotorEx) Fly_Wheel).getVelocity()) < 30) {
                    Ready = Math.abs(Wheel_Speed - ((DcMotorEx) Fly_Wheel).getVelocity());
                } else {
                    Ready = Math.abs(Wheel_Speed - ((DcMotorEx) Fly_Wheel).getVelocity());
                }
                telemetry.addData("Motor Speed", Wheel_Speed);
                telemetry.addData("Motor Power", Fly_Wheel.getPower());
                telemetry.addData("Motor Speed_Measured", ((DcMotorEx) Fly_Wheel).getVelocity() * 2.14);
                telemetry.addData("Status", Ready);
                telemetry.addData("Guard_Angle", Guard_Position);
                telemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void Initialization() {
        LED_DigitalChannel.setMode(DigitalChannel.Mode.OUTPUT);
        LED_DigitalChannel.setState(true);
        Fly_Wheel.setDirection(DcMotor.Direction.REVERSE);
        Fly_Wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Wheel_Speed = 100;
        Fly_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ((DcMotorEx) Fly_Wheel).setVelocity(0);
        Guard_Angle.setDirection(Servo.Direction.FORWARD);
        Guard_Angle.scaleRange(0.27, 1);
        Guard_Position = 0;
        Guard_Angle.setPosition(Guard_Position);
        Shooter_Angle.setDirection(CRServo.Direction.FORWARD);
        Shooter_Angle.setPower(0);
    }

    /**
     * Describe this function...
     */
    private void Set_FLY_Wheel_Speed() {
        ((DcMotorEx) Fly_Wheel).setVelocity(Wheel_Speed);
    }
}