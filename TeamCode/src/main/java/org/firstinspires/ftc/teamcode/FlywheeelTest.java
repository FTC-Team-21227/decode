package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
/**
 * 10/11/25: Runs the flywheel and prints its velocity
 */
public class FlywheeelTest extends LinearOpMode {
    DcMotorEx f;
    public void runOpMode(){
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
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
