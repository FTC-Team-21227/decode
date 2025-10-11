package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class fly extends LinearOpMode {
    DcMotorEx f;
    public void runOpMode(){
        f = hardwareMap.get(DcMotorEx.class, "Fly_Wheel");
        f.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        f.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        waitForStart();
        while (opModeIsActive()){
            f.setPower(-  1);
            telemetry.addLine(""+f.getVelocity());
            telemetry.update();
        }
    }
}
