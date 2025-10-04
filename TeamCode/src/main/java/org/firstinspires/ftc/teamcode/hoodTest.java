package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Hood Test")
// Program used to test hood positions
public class hoodTest extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();
    Hood hood;

    double HoodPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        initialization();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                HoodPosition = 1;
            }
            if (gamepad1.b) {
                HoodPosition = 0;
            }
            if (gamepad1.dpad_up) { // Increase hood position
                HoodPosition += 0.01;
            }
            if (gamepad1.dpad_down) { // Decrease hood position
                HoodPosition -= 0.01;
            }
            hood.turnToAngle(HoodPosition);
            // Telemetry lines
            telemetry.addData("Hood target position", HoodPosition);
            telemetry.addData("Hood curr position", hood.getAngle());
            telemetry.update();
        }
    }

    private void initialization() {
        hood = new Hood(hardwareMap);
        HoodPosition = 0;
    }
}
