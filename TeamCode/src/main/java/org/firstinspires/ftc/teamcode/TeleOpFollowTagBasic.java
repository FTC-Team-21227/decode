package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.AprilTagLocalization.*;
import org.firstinspires.ftc.teamcode.AprilTagLocalization;

import java.util.List;

@TeleOp(name = "TeleOpTag")
public class TeleOpFollowTagBasic extends LinearOpMode {
    //    private MecanumDrive_Lock drive;
    private GoBildaPinpointDriver pinpoint;
    private DcMotor W_BL;
    private DcMotor W_BR;
    private DcMotor W_FR;
    private DcMotor W_FL;
    double Heading_Angle;
    double Motor_power_BR;
    double imu_rotation;
    double Motor_power_BL;
    double Targeting_Angle;
    private AprilTagLocalization april;
    double Motor_fwd_power;
    double Motor_power_FL;
    double Motor_side_power;
    double Motor_power_FR;
    double Motor_Rotation_power;
    double Motor_Power;
    double Motor_Trans_Power;
    double Motor_Rot_Power;
    double currTime = 0;
    double initialHeading;
    boolean back = false;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() throws InterruptedException{
        //getting all the motors, servos, and sensors from the hardware map
        Pose2d initialPose = new Pose2d(0,0,Math.toRadians(0));
        initialHeading = Math.toDegrees(initialPose.heading.toDouble());
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        W_BL = hardwareMap.get(DcMotor.class, "W_BL");
        W_BR = hardwareMap.get(DcMotor.class, "W_BR");
        W_FR = hardwareMap.get(DcMotor.class, "W_FR");
        W_FL = hardwareMap.get(DcMotor.class, "W_FL");

        // Put initialization blocks here.
        Initialization();
        if (opModeIsActive()) {

            // Put run blocks here.
            while (opModeIsActive()) {
                currTime = getRuntime();
                pinpoint.update();

                //Set Targeting_Angle somewhere here to lock onto the apriltag.

                Calculate_IMU_Rotation_Power(); //calculates each motor power based on IMU reading
                Calculate_Motor_Power(); //calculates translational and rotational motor power
                //set power to each wheel motor
                W_BL.setPower(Motor_power_BL);
                W_BR.setPower(Motor_power_BR);
                W_FR.setPower(Motor_power_FR);
                W_FL.setPower(Motor_power_FL);

                if (gamepad1.back && !back) { //change to sub pos for easy align
                    initialHeading = 0;
                    Targeting_Angle = 0;
                    Heading_Angle = 0;
                    pinpoint.resetPosAndIMU();
                    try {
                        Thread.sleep(300);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }

                telemetry.addData("Rotation Power", Motor_Rotation_power);
                telemetry.addData("Motor_Translation Power", Motor_Trans_Power);
                telemetry.addData("Heading", Heading_Angle);
                telemetry.addData("Initial Heading", initialHeading);
                telemetry.addData("Targeting Angle", Targeting_Angle);
                telemetry.addData("Motor Power", Motor_Power);
                telemetry.addData("Side Power", Motor_side_power);
                telemetry.addData("FWD Power", Motor_fwd_power);
                telemetry.addData("IMU_Rotation Power", imu_rotation);
                telemetry.addData("Run time", currTime);
                telemetry.addData("Loop time", getRuntime() - currTime);
                telemetry.update();
            }
        }
    }
    /**
     * Describe this function...
     */
    private void Initialization() {
        W_FR.setDirection(DcMotor.Direction.FORWARD);
        W_FL.setDirection(DcMotor.Direction.REVERSE);
        W_BR.setDirection(DcMotor.Direction.FORWARD);
        W_BL.setDirection(DcMotor.Direction.REVERSE);
        W_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W_FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        W_FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        W_BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        W_BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        W_FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        W_FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        W_BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        W_BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Motor_Power = 1.0;
        Motor_Trans_Power = 1.0;
        Motor_Rot_Power = 1.0;

        Targeting_Angle = initialHeading;
        april = new AprilTagLocalization();

        waitForStart();
    }

    /**
     * Describe this function...
     */
    private void Calculate_IMU_Rotation_Power() {
        double Angle_Difference;
        try {
            Heading_Angle = Math.toDegrees(pinpoint.getHeading(AngleUnit.RADIANS)) + initialHeading; //degrees
        }
        catch (Exception e){
            Heading_Angle = Targeting_Angle;
        }
        if (Math.abs(gamepad1.right_stick_x) >= 0.01) {
            imu_rotation = 0;
            Targeting_Angle = april.getYaw(); // INSERT YAW ANGLE
        }
        //AprilTag angle locking: when the driver isn't turning the robot, lock the robot's heading onto the apriltag.
        else {
            Angle_Difference = Heading_Angle - Targeting_Angle; // <= This is what we're using
            if (Angle_Difference > 180) {
                Angle_Difference = Angle_Difference - 360;
            } else if (Angle_Difference < -180) {
                Angle_Difference = Angle_Difference + 360;
            }
            if (Math.abs(Angle_Difference) < 1) {
                imu_rotation = 0;
            }
            //FOR PROPORTIONAL ANGLE CONTROL: CHANGE THESE TO NONZERO
            else if (Angle_Difference >= 1) {
                imu_rotation = (Angle_Difference * 0.0 /*+ 0.1*/);
            } else {
                imu_rotation = (Angle_Difference * -0.0 /*- 0.1*/);
            }
        }
    }

    /**
     * Describe this function...
     */
    private void Calculate_Motor_Power() {
        double Motor_FWD_input;
        double Motor_Side_input;
        double mag;
        double x;
        double y;
        x = gamepad1.left_stick_x;
        y = gamepad1.left_stick_y;
        mag = Math.sqrt(y*y + x*x);
        Motor_FWD_input = y * mag * mag;
        Motor_Side_input = -x * mag * mag;
        Motor_fwd_power = (Math.cos(Heading_Angle / 180 * Math.PI) * Motor_FWD_input - Math.sin(Heading_Angle / 180 * Math.PI) * Motor_Side_input) * Motor_Trans_Power;
        Motor_side_power = (Math.cos(Heading_Angle / 180 * Math.PI) * Motor_Side_input + Math.sin(Heading_Angle / 180 * Math.PI) * Motor_FWD_input) / 0.7736350635 * Motor_Trans_Power; //*1.5
        Motor_Rotation_power = gamepad1.right_stick_x * 0.35 * Motor_Rot_Power + imu_rotation; //0.7 //0.5
        Motor_power_BL = -(((Motor_fwd_power - Motor_side_power) - Motor_Rotation_power) * Motor_Power);
        Motor_power_BR = -((Motor_fwd_power + Motor_side_power + Motor_Rotation_power) * Motor_Power);
        Motor_power_FL = -(((Motor_fwd_power + Motor_side_power) - Motor_Rotation_power) * Motor_Power);
        Motor_power_FR = -(((Motor_fwd_power - Motor_side_power) + Motor_Rotation_power) * Motor_Power);
        double m = Math.max(Math.max(Math.abs(Motor_power_BL),Math.abs(Motor_power_BR)),Math.max(Math.abs(Motor_power_FL),Math.abs(Motor_power_FR)));
        if (m > 1){
            Motor_power_BL /= m;
            Motor_power_BR /= m;
            Motor_power_FL /= m;
            Motor_power_FR /= m;
        }
    }
}
