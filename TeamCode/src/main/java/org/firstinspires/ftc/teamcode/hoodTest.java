package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Hood Test")
// Program used to test hood positions
public class hoodTest extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();
    Hood hood;

    double HoodAngle;

    @Override
    public void runOpMode() throws InterruptedException {
        initialization();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) { // Lowest angle
                HoodAngle = 0;
            }
            if (gamepad1.yWasPressed()) { // Highest angle (41 degrees)
                HoodAngle = 35;
            }
            if (gamepad1.dpadUpWasPressed()) { // Increase hood position
                HoodAngle += 1;
            }
            if (gamepad1.dpadDownWasPressed()) { // Decrease hood position
                HoodAngle -= 1;
            }
            hood.turnToAngle(HoodAngle);
            // Telemetry lines
            telemetry.addData("Hood target angle", HoodAngle);
            telemetry.addData("Hood curr angle", hood.getAngle());
            telemetry.update();
        }
    }

    private void initialization() {
        hood = new Hood(hardwareMap);
        HoodAngle = 0;
    }
}
